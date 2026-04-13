"""
MapPoint — a single 3D landmark observed across multiple KeyFrames.

Stores
------
  pos          : 3D world position — numpy float32 array, shape (3,)
  descriptor   : representative ORB descriptor — numpy uint8 array, shape (32,)
                 (32 bytes = 256 bits, matching the ORB binary descriptor format)
  observations : {KeyFrame -> keypoint_index}
                 mirrors ORB-SLAM3's unordered_map<KeyFrame*, size_t>
  is_bad       : bool — True once the point is culled
  num_visible  : int  — how many times Tracking predicted this point visible
  num_found    : int  — how many times it was actually matched
                 ratio = num_found / num_visible; points below threshold are culled

Thread safety
-------------
Every public attribute and method is protected by self._lock (threading.Lock).
Callers can also use the context-manager form::

    with map_point:
        # lock is held for the whole block
        ...
"""

import threading
import numpy as np


class MapPoint:
    """A single 3D map landmark."""

    # ---- Class-level unique-ID counter (shared across all instances) ----
    _id_counter: int = 0
    _id_lock: threading.Lock = threading.Lock()

    def __init__(
        self,
        position: np.ndarray,
        descriptor: np.ndarray,
        keyframe,          # KeyFrame that first created this point
        kp_idx: int,       # keypoint index inside that keyframe
    ) -> None:
        """
        Parameters
        ----------
        position   : array-like, shape (3,), float  — world (x, y, z)
        descriptor : array-like, shape (32,), uint8 — 32-byte ORB descriptor
        keyframe   : KeyFrame or None               — reference keyframe
        kp_idx     : int                            — index in keyframe.keypoints
        """
        # Assign a unique, monotonically increasing ID
        with MapPoint._id_lock:
            self.id: int = MapPoint._id_counter
            MapPoint._id_counter += 1

        self._lock: threading.Lock = threading.Lock()

        # ---- 3D world position (numpy float32, shape (3,)) ----
        self._pos: np.ndarray = np.array(position, dtype=np.float32).reshape(3)

        # ---- Representative ORB descriptor (32 bytes = 256-bit binary string) ----
        self._descriptor: np.ndarray = np.array(descriptor, dtype=np.uint8).reshape(32)

        # ---- Observation list: {KeyFrame: keypoint_index} ----
        # A MapPoint needs ≥ 2 observations to be considered stable.
        self._observations: dict = {}
        if keyframe is not None:
            self._observations[keyframe] = kp_idx

        # ---- Quality / culling stats ----
        self._is_bad: bool = False
        self._num_visible: int = 1   # predicted-visible counter
        self._num_found: int = 1     # actually-matched counter

        # Observable distance range (set by Local Mapping after triangulation)
        self._min_distance: float = 0.0
        self._max_distance: float = 0.0

    # ------------------------------------------------------------------
    # Context-manager — `with map_point: ...` holds the lock
    # ------------------------------------------------------------------
    def __enter__(self) -> "MapPoint":
        self._lock.acquire()
        return self

    def __exit__(self, *args) -> None:
        self._lock.release()

    # ------------------------------------------------------------------
    # 3D position
    # ------------------------------------------------------------------
    @property
    def pos(self) -> np.ndarray:
        """Return a copy of the world-space position (float32, shape (3,))."""
        with self._lock:
            return self._pos.copy()

    @pos.setter
    def pos(self, value: np.ndarray) -> None:
        with self._lock:
            self._pos = np.array(value, dtype=np.float32).reshape(3)

    # ------------------------------------------------------------------
    # Descriptor
    # ------------------------------------------------------------------
    @property
    def descriptor(self) -> np.ndarray:
        """Return a copy of the representative 32-byte ORB descriptor."""
        with self._lock:
            return self._descriptor.copy()

    @descriptor.setter
    def descriptor(self, value: np.ndarray) -> None:
        with self._lock:
            self._descriptor = np.array(value, dtype=np.uint8).reshape(32)

    # ------------------------------------------------------------------
    # Observations  {KeyFrame -> keypoint_index}
    # ------------------------------------------------------------------
    def add_observation(self, keyframe, kp_idx: int) -> None:
        """Register that `keyframe` observes this point at `kp_idx`."""
        with self._lock:
            if keyframe not in self._observations:
                self._observations[keyframe] = kp_idx

    def remove_observation(self, keyframe) -> None:
        """
        Remove `keyframe` from the observation list.
        Automatically marks the point as bad if fewer than 2 observers remain.
        """
        with self._lock:
            self._observations.pop(keyframe, None)
            if len(self._observations) < 2:
                self._is_bad = True

    def get_observations(self) -> dict:
        """Return a shallow copy of the observation dict {KeyFrame: kp_idx}."""
        with self._lock:
            return dict(self._observations)

    def num_observations(self) -> int:
        with self._lock:
            return len(self._observations)

    def is_in_keyframe(self, keyframe) -> bool:
        with self._lock:
            return keyframe in self._observations

    def get_index_in_keyframe(self, keyframe) -> int:
        """Return the keypoint index for `keyframe`, or -1 if not observed."""
        with self._lock:
            return self._observations.get(keyframe, -1)

    # ------------------------------------------------------------------
    # Quality flags
    # ------------------------------------------------------------------
    @property
    def is_bad(self) -> bool:
        with self._lock:
            return self._is_bad

    def set_bad(self) -> None:
        """Mark this point as bad and clear all observations."""
        with self._lock:
            self._is_bad = True
            self._observations.clear()

    # ---- Visibility / found counters (used by culling logic in Local Mapping) ----
    def increase_visible(self, n: int = 1) -> None:
        with self._lock:
            self._num_visible += n

    def increase_found(self, n: int = 1) -> None:
        with self._lock:
            self._num_found += n

    @property
    def found_ratio(self) -> float:
        """
        found / visible ratio.
        Local Mapping culls points whose ratio stays below 0.25.
        """
        with self._lock:
            if self._num_visible == 0:
                return 0.0
            return self._num_found / self._num_visible

    # ---- Distance bounds (set after triangulation) ----
    def set_distance_bounds(self, min_dist: float, max_dist: float) -> None:
        with self._lock:
            self._min_distance = float(min_dist)
            self._max_distance = float(max_dist)

    @property
    def min_distance(self) -> float:
        with self._lock:
            return self._min_distance

    @property
    def max_distance(self) -> float:
        with self._lock:
            return self._max_distance

    # ------------------------------------------------------------------
    # Hamming-distance helper (used by descriptor matching)
    # ------------------------------------------------------------------
    @staticmethod
    def hamming_distance(desc_a: np.ndarray, desc_b: np.ndarray) -> int:
        """
        Compute the Hamming distance between two 32-byte ORB descriptors.
        Equivalent to OpenCV's NORM_HAMMING on CV_8U descriptor rows.
        """
        xor = np.bitwise_xor(desc_a, desc_b)
        # Count set bits via the numpy popcount trick
        return int(np.unpackbits(xor).sum())

    # ------------------------------------------------------------------
    # Repr
    # ------------------------------------------------------------------
    def __repr__(self) -> str:
        return (
            f"MapPoint(id={self.id}, "
            f"pos=[{self._pos[0]:.2f},{self._pos[1]:.2f},{self._pos[2]:.2f}], "
            f"obs={len(self._observations)}, bad={self._is_bad})"
        )
