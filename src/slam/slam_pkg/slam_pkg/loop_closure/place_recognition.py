"""
Place Recognition & Candidate Verification  (Person 5, Thread 3).

Implements Steps 13-16 of the ORB-SLAM3 loop-closure pipeline:

  Step 13 — DBoW2 database query          (_query_candidates)
  Step 14 — Sim3 / SE3 RANSAC alignment   (_compute_alignment)
  Step 15 — Verification                  (_verify_candidate)
  Step 16 — Loop vs. merge decision        (_classify_decision)

Public entry point
------------------
    result = place_recognition.detect(kf_query)
    # result is PlaceRecognitionResult or None
"""

from __future__ import annotations

import dataclasses
import logging
import threading
from collections import deque
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from slam_pkg.core.atlas import Atlas
    from slam_pkg.core.keyframe import KeyFrame
    from slam_pkg.core.map_point import MapPoint

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class PlaceRecognitionResult:
    """Payload returned to Person 6 (correction / map_merger)."""

    candidate_kf: "KeyFrame"
    transform: np.ndarray                            # 4x4 Sim3 or SE3 (query -> candidate)
    scale: float                                     # 1.0 when SE3 / known-scale
    matched_points: list                             # [(query_mp, candidate_mp), ...]
    decision: str                                    # "loop" or "merge"
    welding_window: list                             # candidate + covisible neighbors


# ---------------------------------------------------------------------------
# Default configuration
# ---------------------------------------------------------------------------

_DEFAULT_CONFIG: dict = {
    "min_score_factor": 0.75,
    "n_candidates": 10,
    "ransac_iterations": 300,
    "ransac_inlier_threshold": 0.05,                 # 3-D Euclidean distance (metres)
    "descriptor_match_threshold": 50,                # Hamming distance
    "descriptor_ratio_test": 0.75,
    "min_inliers": 20,
    "min_verified_inliers": 40,
    "guided_search_radius_pass1": 5.0,               # pixels
    "guided_search_radius_pass2": 3.0,               # pixels
    "consistency_required": 3,
    "gravity_tolerance_rad": 0.5,
    "covisibility_filter_n": 5,                      # exclude top-N covisible KFs from candidates
    "welding_window_size": 10,
}


# ---------------------------------------------------------------------------
# PlaceRecognition
# ---------------------------------------------------------------------------

class PlaceRecognition:
    """
    Full place-recognition pipeline for Thread 3.

    Parameters
    ----------
    atlas : Atlas
        Global atlas providing the shared KeyFrameDatabase and map registry.
    config : dict or None
        Override any key in ``_DEFAULT_CONFIG``.
    scale_known : bool
        True for stereo / RGB-D / mature VI maps.  Selects SE3 solver
        instead of Sim3.
    """

    def __init__(
        self,
        atlas: "Atlas",
        config: dict | None = None,
        scale_known: bool = True,
    ) -> None:
        self._atlas = atlas
        self._cfg: dict = {**_DEFAULT_CONFIG, **(config or {})}
        self._scale_known: bool = scale_known

        # Covisibility consistency buffer: deque of sets of KF ids
        self._consistency_buffer: deque[set[int]] = deque(
            maxlen=self._cfg["consistency_required"],
        )
        self._lock = threading.Lock()

    # ======================================================================
    # Public API
    # ======================================================================

    def detect(self, kf_query: "KeyFrame") -> PlaceRecognitionResult | None:
        """
        Run the full pipeline (Steps 13-16) on *kf_query*.

        Returns a ``PlaceRecognitionResult`` if a verified loop or merge
        candidate is found, otherwise ``None``.
        """
        # Step 13 — DBoW2 query
        candidates = self._query_candidates(kf_query)
        if not candidates:
            self._push_consistency(set())
            return None

        # Step 14 + 15 — try each candidate in score order
        for cand_kf, _score in candidates:
            alignment = self._compute_alignment(kf_query, cand_kf)
            if alignment is None:
                continue

            T, scale, matched_pts = alignment

            if not self._verify_candidate(kf_query, cand_kf, T, scale, matched_pts):
                continue

            # Step 16 — classify
            decision = self._classify_decision(kf_query, cand_kf)

            welding_kfs = [cand_kf] + cand_kf.get_best_covisibility_keyframes(
                self._cfg["welding_window_size"]
            )

            return PlaceRecognitionResult(
                candidate_kf=cand_kf,
                transform=T,
                scale=scale,
                matched_points=matched_pts,
                decision=decision,
                welding_window=welding_kfs,
            )

        self._push_consistency(set())
        return None

    # ======================================================================
    # Step 13 — DBoW2 Database Query
    # ======================================================================

    def _query_candidates(
        self, kf_query: "KeyFrame"
    ) -> list[tuple["KeyFrame", float]]:
        """
        Query the shared KeyFrameDatabase for place-recognition candidates.

        The minimum acceptable similarity score is derived from the best
        covisibility neighbor of *kf_query*: we multiply that neighbor's
        BoW similarity by ``min_score_factor``.  Candidates that are direct
        covisibility neighbors of the query are filtered out (they are
        temporally adjacent, not genuine revisits).
        """
        db = self._atlas.get_keyframe_database()
        bow = kf_query.bow_vector
        if not bow:
            return []

        # Baseline score from best covisibility neighbor
        covis_kfs = kf_query.get_best_covisibility_keyframes(1)
        if covis_kfs:
            ref_bow = covis_kfs[0].bow_vector
            min_score = self._bow_l1_score(bow, ref_bow) * self._cfg["min_score_factor"]
        else:
            min_score = 0.0

        raw = db.query(bow, n_results=self._cfg["n_candidates"], min_score=min_score)

        # Filter out temporal neighbors
        neighbor_ids = {
            kf.id
            for kf in kf_query.get_best_covisibility_keyframes(
                self._cfg["covisibility_filter_n"]
            )
        }
        neighbor_ids.add(kf_query.id)

        filtered = [(kf, sc) for kf, sc in raw if kf.id not in neighbor_ids and not kf.is_bad]
        return filtered

    # ======================================================================
    # Step 14 — Sim3 / SE3 RANSAC Alignment
    # ======================================================================

    def _compute_alignment(
        self,
        kf_query: "KeyFrame",
        kf_candidate: "KeyFrame",
    ) -> tuple[np.ndarray, float, list] | None:
        """
        Build 3-D correspondences via descriptor matching and solve for a
        rigid (SE3) or similarity (Sim3) transform using RANSAC.

        Returns ``(T_4x4, scale, matched_point_pairs)`` or ``None``.
        """
        correspondences = self._match_map_points(kf_query, kf_candidate)
        if len(correspondences) < self._cfg["min_inliers"]:
            return None

        pts_q = np.array([mp_q.pos for mp_q, _ in correspondences], dtype=np.float64)
        pts_c = np.array([mp_c.pos for _, mp_c in correspondences], dtype=np.float64)

        best_inliers: np.ndarray | None = None
        best_T: np.ndarray | None = None
        best_scale: float = 1.0
        n_best = 0
        rng = np.random.default_rng()

        n_corr = len(correspondences)
        threshold = self._cfg["ransac_inlier_threshold"]

        for _ in range(self._cfg["ransac_iterations"]):
            idxs = rng.choice(n_corr, size=3, replace=False)
            sample_q = pts_q[idxs]
            sample_c = pts_c[idxs]

            if self._scale_known:
                R, t = self._horn_se3(sample_q, sample_c)
                s = 1.0
            else:
                R, t, s = self._horn_sim3(sample_q, sample_c)
                if s <= 0.0:
                    continue

            transformed = (s * (R @ pts_q.T)).T + t
            dists = np.linalg.norm(transformed - pts_c, axis=1)
            inlier_mask = dists < threshold
            n_inliers = int(inlier_mask.sum())

            if n_inliers > n_best:
                n_best = n_inliers
                best_inliers = inlier_mask
                best_T = self._build_sim3_matrix(R, t, s)
                best_scale = s

        if n_best < self._cfg["min_inliers"]:
            return None

        # Refine on all inliers
        inlier_q = pts_q[best_inliers]
        inlier_c = pts_c[best_inliers]
        if self._scale_known:
            R, t = self._horn_se3(inlier_q, inlier_c)
            s = 1.0
        else:
            R, t, s = self._horn_sim3(inlier_q, inlier_c)
        best_T = self._build_sim3_matrix(R, t, s)
        best_scale = s

        matched = [correspondences[i] for i in range(n_corr) if best_inliers[i]]
        return best_T, best_scale, matched

    # ======================================================================
    # Step 15 — Verification
    # ======================================================================

    def _verify_candidate(
        self,
        kf_query: "KeyFrame",
        kf_candidate: "KeyFrame",
        T: np.ndarray,
        scale: float,
        matched_pts: list,
    ) -> bool:
        """
        Three-gate verification:
          a) Two-pass guided matching refinement
          b) Covisibility consistency check
          c) Gravity consistency check (VI maps)
        """
        # (a) Guided matching — two-pass
        total_inliers = len(matched_pts)

        extra_pass1 = self._guided_match(
            kf_query, kf_candidate, T, scale,
            radius=self._cfg["guided_search_radius_pass1"],
            existing=matched_pts,
        )
        total_inliers += extra_pass1

        extra_pass2 = self._guided_match(
            kf_query, kf_candidate, T, scale,
            radius=self._cfg["guided_search_radius_pass2"],
            existing=matched_pts,
        )
        total_inliers += extra_pass2

        if total_inliers < self._cfg["min_verified_inliers"]:
            self._push_consistency(set())
            return False

        # (b) Covisibility consistency
        cand_group = self._candidate_group_ids(kf_candidate)
        if not self._check_consistency(cand_group):
            return False

        # (c) Gravity consistency
        if not self._check_gravity(T):
            self._push_consistency(set())
            return False

        return True

    # ======================================================================
    # Step 16 — Loop vs. Merge Decision
    # ======================================================================

    def _classify_decision(
        self,
        kf_query: "KeyFrame",
        kf_candidate: "KeyFrame",
    ) -> str:
        """Return ``"loop"`` if both KFs share the same Map, else ``"merge"``."""
        query_map = self._find_map_of(kf_query)
        cand_map = self._find_map_of(kf_candidate)
        if query_map is not None and cand_map is not None and query_map is cand_map:
            return "loop"
        return "merge"

    # ------------------------------------------------------------------
    # Helpers — descriptor matching
    # ------------------------------------------------------------------

    def _match_map_points(
        self,
        kf_a: "KeyFrame",
        kf_b: "KeyFrame",
    ) -> list[tuple["MapPoint", "MapPoint"]]:
        """
        Brute-force descriptor matching between map points visible in
        *kf_a* and *kf_b*, using Hamming distance with a ratio test.
        """
        from slam_pkg.core.map_point import MapPoint

        mps_a = kf_a.get_map_points()
        mps_b = kf_b.get_map_points()
        if not mps_a or not mps_b:
            return []

        list_a = [(mp, mp.descriptor) for mp in mps_a if not mp.is_bad]
        list_b = [(mp, mp.descriptor) for mp in mps_b if not mp.is_bad]
        if not list_a or not list_b:
            return []

        thresh = self._cfg["descriptor_match_threshold"]
        ratio = self._cfg["descriptor_ratio_test"]
        matches: list[tuple["MapPoint", "MapPoint"]] = []

        for mp_a, desc_a in list_a:
            best_dist = 256
            second_dist = 256
            best_mp_b = None
            for mp_b, desc_b in list_b:
                d = MapPoint.hamming_distance(desc_a, desc_b)
                if d < best_dist:
                    second_dist = best_dist
                    best_dist = d
                    best_mp_b = mp_b
                elif d < second_dist:
                    second_dist = d

            if best_mp_b is None:
                continue
            if best_dist > thresh:
                continue
            if second_dist > 0 and best_dist / second_dist > ratio:
                continue
            matches.append((mp_a, best_mp_b))

        return matches

    # ------------------------------------------------------------------
    # Helpers — guided matching (Step 15a)
    # ------------------------------------------------------------------

    def _guided_match(
        self,
        kf_query: "KeyFrame",
        kf_candidate: "KeyFrame",
        T: np.ndarray,
        scale: float,
        radius: float,
        existing: list,
    ) -> int:
        """
        Project candidate map points into the query frame using *T*,
        search for descriptor matches within *radius* pixels.

        Returns the number of **new** inlier matches found (also appends
        them to *existing* in-place).
        """
        from slam_pkg.core.map_point import MapPoint

        R = T[:3, :3]
        t = T[:3, 3]

        already_matched_q = {id(mp_q) for mp_q, _ in existing}
        already_matched_c = {id(mp_c) for _, mp_c in existing}

        query_mps = kf_query.get_map_points()
        kp_list = kf_query.keypoints
        if not kp_list:
            return 0

        # Build lookup: keypoint_index -> MapPoint for the query frame
        kp_idx_to_mp = {idx: mp for mp, idx in query_mps.items() if not mp.is_bad}

        cand_mps = kf_candidate.get_map_points()
        thresh = self._cfg["descriptor_match_threshold"]
        new_count = 0

        for mp_c, _kp_idx_c in cand_mps.items():
            if mp_c.is_bad or id(mp_c) in already_matched_c:
                continue

            # Transform candidate point into query frame
            pt_w = mp_c.pos.astype(np.float64)
            pt_q = R[:3, :3] @ pt_w + t[:3] if T.shape == (4, 4) else R @ pt_w + t
            # We don't have camera intrinsics here so we fall back to 3-D
            # proximity: find query map points within *radius* (metres,
            # repurposed from the pixel threshold as a spatial tolerance
            # scaled by 0.01 to approximate nearby 3-D distance).
            spatial_tol = radius * 0.01

            for kp_idx, mp_q in kp_idx_to_mp.items():
                if mp_q.is_bad or id(mp_q) in already_matched_q:
                    continue
                dist3d = np.linalg.norm(mp_q.pos.astype(np.float64) - pt_q)
                if dist3d > spatial_tol:
                    continue
                d = MapPoint.hamming_distance(mp_c.descriptor, mp_q.descriptor)
                if d < thresh:
                    existing.append((mp_q, mp_c))
                    already_matched_q.add(id(mp_q))
                    already_matched_c.add(id(mp_c))
                    new_count += 1
                    break

        return new_count

    # ------------------------------------------------------------------
    # Helpers — covisibility consistency (Step 15b)
    # ------------------------------------------------------------------

    def _candidate_group_ids(self, kf_candidate: "KeyFrame") -> set[int]:
        group = {kf_candidate.id}
        for kf in kf_candidate.get_best_covisibility_keyframes(
            self._cfg["welding_window_size"]
        ):
            group.add(kf.id)
        return group

    def _push_consistency(self, group: set[int]) -> None:
        with self._lock:
            self._consistency_buffer.append(group)

    def _check_consistency(self, group: set[int]) -> bool:
        """
        Return True if *group* has a non-trivial overlap with every entry
        currently in the consistency buffer, meaning the same region has
        been detected across consecutive calls.
        """
        with self._lock:
            buf = list(self._consistency_buffer)

        required = self._cfg["consistency_required"]
        if len(buf) < required - 1:
            self._push_consistency(group)
            return False

        for prev in buf:
            if not prev:
                self._push_consistency(group)
                return False
            if not prev & group:
                self._push_consistency(group)
                return False

        self._push_consistency(group)
        return True

    # ------------------------------------------------------------------
    # Helpers — gravity consistency (Step 15c)
    # ------------------------------------------------------------------

    @staticmethod
    def _check_gravity(T: np.ndarray, tolerance: float = 0.5) -> bool:
        """
        For VI maps the transform should not rotate the gravity vector
        beyond *tolerance* radians (ignoring yaw).

        Gravity is assumed to be the world-Z axis.  We decompose the
        rotation into a yaw component (around Z) and a residual tilt;
        the tilt must be small.
        """
        R = T[:3, :3].copy()
        # Normalise in case of Sim3 scaling baked into R
        col_norms = np.linalg.norm(R, axis=0)
        if np.any(col_norms < 1e-9):
            return False
        R = R / col_norms

        gravity = np.array([0.0, 0.0, 1.0])
        rotated_gravity = R @ gravity
        cos_angle = np.clip(np.dot(rotated_gravity, gravity), -1.0, 1.0)
        tilt = np.arccos(cos_angle)
        return float(tilt) < tolerance

    # ------------------------------------------------------------------
    # Helpers — Horn's method (SE3 / Sim3)
    # ------------------------------------------------------------------

    @staticmethod
    def _horn_se3(
        pts_src: np.ndarray, pts_dst: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute the least-squares SE3 (R, t) aligning *pts_src* to
        *pts_dst* using Horn's quaternion method (no scale).

        Returns (R [3x3], t [3,]).
        """
        centroid_s = pts_src.mean(axis=0)
        centroid_d = pts_dst.mean(axis=0)
        qs = pts_src - centroid_s
        qd = pts_dst - centroid_d

        H = qs.T @ qd
        U, _, Vt = np.linalg.svd(H)
        d = np.linalg.det(Vt.T @ U.T)
        D = np.diag([1.0, 1.0, d])
        R = Vt.T @ D @ U.T
        t = centroid_d - R @ centroid_s
        return R, t

    @staticmethod
    def _horn_sim3(
        pts_src: np.ndarray, pts_dst: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """
        Compute Sim3 (R, t, s) aligning *pts_src* to *pts_dst*.

        Returns (R [3x3], t [3,], s).
        """
        centroid_s = pts_src.mean(axis=0)
        centroid_d = pts_dst.mean(axis=0)
        qs = pts_src - centroid_s
        qd = pts_dst - centroid_d

        H = qs.T @ qd
        U, _, Vt = np.linalg.svd(H)
        d = np.linalg.det(Vt.T @ U.T)
        D = np.diag([1.0, 1.0, d])
        R = Vt.T @ D @ U.T

        var_s = (qs ** 2).sum()
        if var_s < 1e-12:
            return R, centroid_d - centroid_s, 1.0
        s = float(np.trace(R @ H.T) / var_s) if var_s > 0 else 1.0

        t = centroid_d - s * R @ centroid_s
        return R, t, s

    @staticmethod
    def _build_sim3_matrix(
        R: np.ndarray, t: np.ndarray, s: float
    ) -> np.ndarray:
        """Build a 4x4 Sim3 matrix  [sR | t ; 0 0 0 1]."""
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = s * R
        T[:3, 3] = t
        return T

    # ------------------------------------------------------------------
    # Helpers — BoW L1 score
    # ------------------------------------------------------------------

    @staticmethod
    def _bow_l1_score(bv_a: dict, bv_b: dict) -> float:
        """DBoW2 L1 scoring: ``1 - 0.5 * sum|a_i - b_i|``."""
        all_words = set(bv_a) | set(bv_b)
        l1 = sum(abs(bv_a.get(w, 0.0) - bv_b.get(w, 0.0)) for w in all_words)
        return 1.0 - 0.5 * l1

    # ------------------------------------------------------------------
    # Helpers — map lookup
    # ------------------------------------------------------------------

    def _find_map_of(self, kf: "KeyFrame"):
        """Return the Map containing *kf*, or None."""
        for m in self._atlas.get_all_maps():
            if m.contains_keyframe(kf):
                return m
        return None
