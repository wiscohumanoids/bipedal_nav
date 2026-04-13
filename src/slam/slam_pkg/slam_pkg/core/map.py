"""
Map — container for a single mapping session.

Each Map owns:
  - A set of KeyFrame objects
  - A set of MapPoint objects

The Atlas manages a list of Maps (one active, the rest stored).
Thread 2 (Local Mapping) is the primary writer; Threads 1 and 3 read.

Thread safety
-------------
All public methods acquire self._lock before touching internal state.
"""

import threading


class Map:
    """Container for the KeyFrames and MapPoints of one mapping session."""

    # ---- Class-level unique-ID counter ----
    _id_counter: int = 0
    _id_lock: threading.Lock = threading.Lock()

    def __init__(self) -> None:
        with Map._id_lock:
            self.id: int = Map._id_counter
            Map._id_counter += 1

        self._lock: threading.Lock = threading.Lock()

        # Internal storage — sets give O(1) add / discard
        self._keyframes: set = set()
        self._map_points: set = set()

        # Whether this is the map Tracking is currently writing to
        self._is_active: bool = False

    # ==================================================================
    # KeyFrames
    # ==================================================================
    def add_keyframe(self, kf) -> None:
        """Add a KeyFrame to this map."""
        with self._lock:
            self._keyframes.add(kf)

    def remove_keyframe(self, kf) -> None:
        """Remove a KeyFrame (called when culled by Local Mapping)."""
        with self._lock:
            self._keyframes.discard(kf)

    def get_all_keyframes(self) -> list:
        """Return a snapshot list of all KeyFrames in this map."""
        with self._lock:
            return list(self._keyframes)

    def num_keyframes(self) -> int:
        with self._lock:
            return len(self._keyframes)

    def contains_keyframe(self, kf) -> bool:
        with self._lock:
            return kf in self._keyframes

    # ==================================================================
    # MapPoints
    # ==================================================================
    def add_map_point(self, mp) -> None:
        """Add a MapPoint to this map."""
        with self._lock:
            self._map_points.add(mp)

    def remove_map_point(self, mp) -> None:
        """Remove a MapPoint (called when culled or merged)."""
        with self._lock:
            self._map_points.discard(mp)

    def get_all_map_points(self) -> list:
        """Return a snapshot list of all MapPoints in this map."""
        with self._lock:
            return list(self._map_points)

    def num_map_points(self) -> int:
        with self._lock:
            return len(self._map_points)

    def contains_map_point(self, mp) -> bool:
        with self._lock:
            return mp in self._map_points

    # ==================================================================
    # Status
    # ==================================================================
    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._is_active

    @is_active.setter
    def is_active(self, value: bool) -> None:
        with self._lock:
            self._is_active = value

    # ==================================================================
    # Repr
    # ==================================================================
    def __repr__(self) -> str:
        return (
            f"Map(id={self.id}, "
            f"active={self._is_active}, "
            f"keyframes={len(self._keyframes)}, "
            f"map_points={len(self._map_points)})"
        )
