"""
KeyFrame — a selected video frame whose pose is permanently stored in the map.

Stores
------
  pose                  : 4×4 float32 numpy array (camera-to-world SE3 transform)
  velocity              : 3-vector float32 — world-frame velocity (VI mode only)
  bias                  : 6-vector float32 — [acc_bias(3), gyro_bias(3)] (VI mode only)
  keypoints             : list[cv2.KeyPoint] — 2D detected feature locations
  descriptors           : np.ndarray shape (N, 32) uint8 — one ORB row per keypoint
  bow_vector            : dict {word_id(int): weight(float)} — sparse DBoW2 BoW vector
  covisibility          : dict {KeyFrame: shared_map_point_count(int)}
                          ← this IS the covisibility graph (no separate structure needed)
  parent                : KeyFrame or None  ← spanning tree edge (single pointer)
  children              : set[KeyFrame]     ← spanning tree children
  map_points            : dict {MapPoint: keypoint_index(int)}

Three graphs — all implicit, no separate containers
-----------------------------------------------------
  Covisibility graph  → lives in self._covisibility  (neighbour → weight)
  Spanning tree       → lives in self._parent + self._children
  Essential graph     → filtered view of covisibility: weight ≥ 100
                        computed on demand via get_essential_graph_neighbors()

Thread safety
-------------
Every public method acquires self._lock before touching internal state.
Use the context-manager form for multi-step atomic operations::

    with keyframe:
        keyframe._pose = ...
        keyframe._velocity = ...
"""

import threading
import numpy as np


class KeyFrame:
    """A selected frame with permanent pose storage in the active map."""

    # ---- Class-level unique-ID counter ----
    _id_counter: int = 0
    _id_lock: threading.Lock = threading.Lock()

    # Minimum shared-point count to appear in the Essential Graph
    ESSENTIAL_GRAPH_THRESHOLD: int = 100

    def __init__(
        self,
        pose: np.ndarray,
        keypoints: list,
        descriptors: np.ndarray,
        bow_vector: dict = None,
        timestamp: float = 0.0,
    ) -> None:
        """
        Parameters
        ----------
        pose        : array-like, shape (4,4), float — camera-to-world SE3
        keypoints   : list[cv2.KeyPoint]             — 2D detected keypoints
        descriptors : np.ndarray, shape (N, 32) uint8 — ORB descriptor matrix
        bow_vector  : dict {int: float} or None       — precomputed BoW vector
        timestamp   : float                           — ROS timestamp (seconds)
        """
        with KeyFrame._id_lock:
            self.id: int = KeyFrame._id_counter
            KeyFrame._id_counter += 1

        self._lock: threading.Lock = threading.Lock()
        self.timestamp: float = float(timestamp)

        # ----------------------------------------------------------------
        # Pose  (SE3, camera-to-world, 4×4 float32)
        # ----------------------------------------------------------------
        self._pose: np.ndarray = np.array(pose, dtype=np.float32).reshape(4, 4)

        # ----------------------------------------------------------------
        # Visual-Inertial extras  (zero-initialised; set by IMU Initializer)
        # ----------------------------------------------------------------
        self._velocity: np.ndarray = np.zeros(3, dtype=np.float32)
        self._bias: np.ndarray = np.zeros(6, dtype=np.float32)  # [acc(3)|gyro(3)]

        # ----------------------------------------------------------------
        # ORB features
        # ----------------------------------------------------------------
        self._keypoints: list = list(keypoints)
        self._descriptors: np.ndarray = (
            np.array(descriptors, dtype=np.uint8)
            if descriptors is not None and len(descriptors) > 0
            else np.empty((0, 32), dtype=np.uint8)
        )

        # ----------------------------------------------------------------
        # BoW vector  {word_id: weight}  — filled by KeyFrameDatabase.add()
        # ----------------------------------------------------------------
        self._bow_vector: dict = dict(bow_vector) if bow_vector else {}

        # ----------------------------------------------------------------
        # Covisibility graph
        # dict {KeyFrame -> shared_map_point_count}
        # Rebuilt every time a MapPoint observation is added/removed.
        # ----------------------------------------------------------------
        self._covisibility: dict = {}

        # ----------------------------------------------------------------
        # Spanning tree
        # Each KeyFrame stores exactly one parent pointer (or None for root).
        # Children are also tracked so re-parenting on KF removal is O(degree).
        # ----------------------------------------------------------------
        self._parent: "KeyFrame | None" = None
        self._children: set = set()

        # ----------------------------------------------------------------
        # MapPoint observations  {MapPoint -> keypoint_index}
        # ----------------------------------------------------------------
        self._map_points: dict = {}

        # ----------------------------------------------------------------
        # Status
        # ----------------------------------------------------------------
        self._is_bad: bool = False

    # ------------------------------------------------------------------
    # Context-manager  — `with keyframe: ...` holds the lock
    # ------------------------------------------------------------------
    def __enter__(self) -> "KeyFrame":
        self._lock.acquire()
        return self

    def __exit__(self, *args) -> None:
        self._lock.release()

    # ==================================================================
    # Pose
    # ==================================================================
    @property
    def pose(self) -> np.ndarray:
        """Return a copy of the 4×4 camera-to-world SE3 matrix."""
        with self._lock:
            return self._pose.copy()

    @pose.setter
    def pose(self, T: np.ndarray) -> None:
        with self._lock:
            self._pose = np.array(T, dtype=np.float32).reshape(4, 4)

    def get_rotation(self) -> np.ndarray:
        """Return the 3×3 rotation sub-matrix (copy)."""
        with self._lock:
            return self._pose[:3, :3].copy()

    def get_translation(self) -> np.ndarray:
        """Return the 3-vector translation (camera-center position in world)."""
        with self._lock:
            return self._pose[:3, 3].copy()

    def get_camera_center(self) -> np.ndarray:
        """
        Return the optical center in world coordinates.
        For a camera-to-world transform T = [R | t], center = t.
        """
        return self.get_translation()

    # ---- Visual-Inertial extras ----
    @property
    def velocity(self) -> np.ndarray:
        with self._lock:
            return self._velocity.copy()

    @velocity.setter
    def velocity(self, v: np.ndarray) -> None:
        with self._lock:
            self._velocity = np.array(v, dtype=np.float32).reshape(3)

    @property
    def bias(self) -> np.ndarray:
        """Return the IMU bias vector [acc_bias(3) | gyro_bias(3)]."""
        with self._lock:
            return self._bias.copy()

    @bias.setter
    def bias(self, b: np.ndarray) -> None:
        with self._lock:
            self._bias = np.array(b, dtype=np.float32).reshape(6)

    # ==================================================================
    # ORB features
    # ==================================================================
    @property
    def keypoints(self) -> list:
        with self._lock:
            return list(self._keypoints)

    @property
    def descriptors(self) -> np.ndarray:
        """Return a copy of the N×32 uint8 descriptor matrix."""
        with self._lock:
            return self._descriptors.copy()

    def num_keypoints(self) -> int:
        with self._lock:
            return len(self._keypoints)

    # ==================================================================
    # BoW vector
    # ==================================================================
    @property
    def bow_vector(self) -> dict:
        with self._lock:
            return dict(self._bow_vector)

    @bow_vector.setter
    def bow_vector(self, bv: dict) -> None:
        with self._lock:
            self._bow_vector = dict(bv)

    # ==================================================================
    # Covisibility graph
    # (lives entirely inside these methods — no separate data structure)
    # ==================================================================
    def update_covisibility(self, other_kf: "KeyFrame", shared_count: int) -> None:
        """
        Set or update the covisibility edge weight between self and `other_kf`.
        Call this whenever a MapPoint observation is added or removed.
        An edge with weight 0 is deleted (no connection).
        """
        with self._lock:
            if shared_count > 0:
                self._covisibility[other_kf] = shared_count
            else:
                self._covisibility.pop(other_kf, None)

    def get_covisibility_keyframes(self, min_weight: int = 0) -> list:
        """
        Return [(KeyFrame, weight), ...] sorted by descending weight.
        Optionally filter to edges with weight >= min_weight.
        """
        with self._lock:
            items = [
                (kf, w)
                for kf, w in self._covisibility.items()
                if w >= min_weight
            ]
        items.sort(key=lambda x: x[1], reverse=True)
        return items

    def get_best_covisibility_keyframes(self, n: int) -> list:
        """Return the top-N most covisible KeyFrames (no weights)."""
        return [kf for kf, _ in self.get_covisibility_keyframes()[:n]]

    def get_covisibility_weight(self, other_kf: "KeyFrame") -> int:
        """Return the shared map-point count with `other_kf` (0 if no edge)."""
        with self._lock:
            return self._covisibility.get(other_kf, 0)

    def remove_covisibility(self, other_kf: "KeyFrame") -> None:
        with self._lock:
            self._covisibility.pop(other_kf, None)

    # ==================================================================
    # Essential graph
    # (sparse sub-graph of covisibility — no extra storage, filtered at query time)
    # ==================================================================
    def get_essential_graph_neighbors(
        self, threshold: int = ESSENTIAL_GRAPH_THRESHOLD
    ) -> list:
        """
        Return KeyFrames whose covisibility edge weight ≥ `threshold`.

        ORB-SLAM3 uses threshold = 100 shared map points.
        This is the only accessor needed; the Essential Graph is never stored
        separately — it is always derived from the covisibility map on demand.
        """
        with self._lock:
            return [
                kf
                for kf, w in self._covisibility.items()
                if w >= threshold
            ]

    # ==================================================================
    # Spanning tree
    # (one parent pointer per KeyFrame — no separate adjacency structure)
    # ==================================================================
    @property
    def parent(self) -> "KeyFrame | None":
        with self._lock:
            return self._parent

    def set_parent(self, kf: "KeyFrame | None") -> None:
        """
        Set the spanning tree parent.
        Also registers self as a child of `kf` (if kf is not None).
        """
        with self._lock:
            # Detach from old parent
            if self._parent is not None and self._parent is not kf:
                self._parent.remove_child(self)
            self._parent = kf
        # Register with new parent (outside our lock to avoid deadlock)
        if kf is not None:
            kf._register_child(self)

    def _register_child(self, kf: "KeyFrame") -> None:
        """Internal — add `kf` to children set. Called by set_parent()."""
        with self._lock:
            self._children.add(kf)

    def remove_child(self, kf: "KeyFrame") -> None:
        with self._lock:
            self._children.discard(kf)

    def get_children(self) -> set:
        """Return a copy of the children set."""
        with self._lock:
            return set(self._children)

    # ==================================================================
    # MapPoint observations  {MapPoint -> keypoint_index}
    # ==================================================================
    def add_map_point(self, mp, kp_idx: int) -> None:
        """Record that MapPoint `mp` is observed at keypoint index `kp_idx`."""
        with self._lock:
            self._map_points[mp] = kp_idx

    def remove_map_point(self, mp) -> None:
        with self._lock:
            self._map_points.pop(mp, None)

    def get_map_point_at(self, kp_idx: int):
        """Return the MapPoint associated with `kp_idx`, or None."""
        with self._lock:
            for mp, idx in self._map_points.items():
                if idx == kp_idx:
                    return mp
        return None

    def get_map_points(self) -> dict:
        """Return a shallow copy of {MapPoint: kp_idx}."""
        with self._lock:
            return dict(self._map_points)

    def num_map_points(self) -> int:
        with self._lock:
            return len(self._map_points)

    # ==================================================================
    # Status
    # ==================================================================
    @property
    def is_bad(self) -> bool:
        with self._lock:
            return self._is_bad

    def set_bad(self) -> None:
        """
        Mark this KeyFrame as bad (culled).
        Local Mapping and Loop Closure call this during redundancy pruning.
        """
        with self._lock:
            self._is_bad = True

    # ------------------------------------------------------------------
    # Repr
    # ------------------------------------------------------------------
    def __repr__(self) -> str:
        return (
            f"KeyFrame(id={self.id}, ts={self.timestamp:.3f}, "
            f"kps={len(self._keypoints)}, "
            f"covis={len(self._covisibility)}, "
            f"mps={len(self._map_points)}, "
            f"bad={self._is_bad})"
        )
