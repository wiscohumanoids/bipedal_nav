"""
Atlas — top-level container managing all Maps and the shared KeyFrameDatabase.

Role in the system
------------------
Atlas is the single source of truth for map state.  Every other component
(Tracking, Local Mapping, Loop Closure) holds a reference to the Atlas and
calls its methods rather than manipulating maps directly.

Responsibilities
----------------
  create_new_map()              : start a new Map and set it as active
                                  (called on startup and on long-term tracking loss)
  store_map(m)                  : deactivate a map without discarding it
  get_active_map()              : return the Map that Tracking is writing to
  merge_maps(map_a, map_b)      : fold map_b into map_a after loop/merge verification
                                  (called by Thread 3 — Loop Closure / Map Merging)
  get_keyframe_database()       : return the ONE shared DBoW2 database
  get_all_keyframes()           : convenience — every KF across ALL maps
  get_all_map_points()          : convenience — every MP across ALL maps

One shared KeyFrameDatabase
---------------------------
The Atlas owns a single KeyFrameDatabase that indexes KeyFrames from ALL
maps.  This is what allows Loop Closure to detect revisited places even
across separate mapping sessions stored in different Map objects.

Thread safety
-------------
Atlas-level operations (create_new_map, store_map, merge_maps) are
protected by self._lock.  Individual Map and KeyFrame accesses use their
own locks (see map.py, keyframe.py).
"""

import threading
from .map import Map
from .keyframe_database import KeyFrameDatabase


class Atlas:
    """Top-level manager for all Maps and the shared KeyFrameDatabase."""

    def __init__(self, vocabulary_path: str = None) -> None:
        """
        Parameters
        ----------
        vocabulary_path : str or None
            Path to a pre-trained ORB vocabulary file (.txt or .bin).
            Passed through to KeyFrameDatabase.
            Set to None to use the pure-Python fallback (no native DBoW2).
        """
        self._lock: threading.Lock = threading.Lock()

        # ---- One shared DBoW2 database for ALL maps ----
        # Keeping a single database means Loop Closure can match across
        # separate sessions (e.g., two runs of the same environment).
        self._kf_database: KeyFrameDatabase = KeyFrameDatabase(vocabulary_path)

        # ---- All maps ever created:  {map_id (int) -> Map} ----
        self._maps: dict = {}

        # ---- Currently active map (the one Tracking writes to) ----
        self._active_map: "Map | None" = None

    # ==================================================================
    # Map lifecycle
    # ==================================================================
    def create_new_map(self) -> Map:
        """
        Create a new, empty Map and set it as active.

        Any previously active Map is deactivated (stored, not discarded)
        so Loop Closure can still find matches against its KeyFrames.

        Returns the newly created Map.
        """
        with self._lock:
            # Deactivate the current map, if any
            if self._active_map is not None:
                self._active_map.is_active = False

            new_map = Map()
            new_map.is_active = True
            self._maps[new_map.id] = new_map
            self._active_map = new_map
            return new_map

    def store_map(self, m: Map) -> None:
        """
        Deactivate `m` without removing it from the Atlas.

        Called when Tracking decides a Map is no longer usable as the
        active session (long-term recovery path).
        """
        with self._lock:
            m.is_active = False
            if self._active_map is m:
                self._active_map = None

    def get_active_map(self) -> "Map | None":
        """Return the currently active Map (or None if none exists yet)."""
        with self._lock:
            return self._active_map

    def set_active_map(self, m: Map) -> None:
        """
        Promote an existing stored Map back to active status.
        Used by Loop Closure after a successful welding / merge.
        """
        with self._lock:
            if self._active_map is not None and self._active_map is not m:
                self._active_map.is_active = False
            m.is_active = True
            self._active_map = m

    def get_all_maps(self) -> list:
        """Return a snapshot list of all Maps (active + stored)."""
        with self._lock:
            return list(self._maps.values())

    def num_maps(self) -> int:
        with self._lock:
            return len(self._maps)

    # ==================================================================
    # Map merging  (called by Thread 3 — Loop Closure / Map Merging)
    # ==================================================================
    def merge_maps(self, map_a: Map, map_b: Map) -> Map:
        """
        Merge map_b INTO map_a.

        Transfers all KeyFrames and MapPoints from map_b to map_a, then
        removes map_b from the Atlas registry.

        The caller (Thread 3) is responsible for:
          - fusing duplicate MapPoints (Welding BA)
          - running Essential Graph optimisation
          - refreshing covisibility + spanning tree

        Parameters
        ----------
        map_a : Map — the surviving map (absorbs map_b)
        map_b : Map — the map being folded in (removed afterwards)

        Returns
        -------
        map_a (the surviving, now-larger map)
        """
        with self._lock:
            # Transfer KeyFrames
            for kf in map_b.get_all_keyframes():
                map_a.add_keyframe(kf)

            # Transfer MapPoints
            for mp in map_b.get_all_map_points():
                map_a.add_map_point(mp)

            # Remove map_b from the registry
            self._maps.pop(map_b.id, None)

            # If map_b was the active map, promote map_a
            if self._active_map is map_b:
                map_a.is_active = True
                self._active_map = map_a

        return map_a

    # ==================================================================
    # KeyFrame Database  (shared across all maps)
    # ==================================================================
    def get_keyframe_database(self) -> KeyFrameDatabase:
        """Return the single shared DBoW2 KeyFrameDatabase."""
        return self._kf_database

    # ==================================================================
    # Convenience accessors (span ALL maps)
    # ==================================================================
    def get_all_keyframes(self) -> list:
        """Return every KeyFrame across all maps (active + stored)."""
        kfs = []
        with self._lock:
            maps = list(self._maps.values())
        for m in maps:
            kfs.extend(m.get_all_keyframes())
        return kfs

    def get_all_map_points(self) -> list:
        """Return every MapPoint across all maps (active + stored)."""
        mps = []
        with self._lock:
            maps = list(self._maps.values())
        for m in maps:
            mps.extend(m.get_all_map_points())
        return mps

    def total_keyframes(self) -> int:
        return len(self.get_all_keyframes())

    def total_map_points(self) -> int:
        return len(self.get_all_map_points())

    # ==================================================================
    # Repr
    # ==================================================================
    def __repr__(self) -> str:
        active_id = self._active_map.id if self._active_map else None
        return (
            f"Atlas("
            f"maps={len(self._maps)}, "
            f"active_map={active_id}, "
            f"db_backend='{self._kf_database.backend}', "
            f"total_kfs={self.total_keyframes()}, "
            f"total_mps={self.total_map_points()})"
        )
