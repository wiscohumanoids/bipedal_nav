"""
core — Global SLAM data structures shared by all three threads.

Classes
-------
MapPoint
    A single 3D landmark: world position (float32 x/y/z), 32-byte ORB
    descriptor, observation list {KeyFrame -> keypoint_index}, quality flags.

KeyFrame
    A selected frame permanently stored in the map: SE3 pose (4×4 float32),
    ORB keypoints + descriptor matrix, BoW vector, covisibility connections,
    spanning-tree parent pointer, MapPoint observation list.
    The three SLAM graphs are all implicit inside KeyFrame:
      - Covisibility graph  → self._covisibility  {KeyFrame: weight}
      - Spanning tree       → self._parent / self._children
      - Essential graph     → covisibility filtered at weight ≥ 100

Map
    Container for one mapping session: owns a set of KeyFrames and a set
    of MapPoints.  The Atlas manages a list of Maps.

Atlas
    Top-level manager: vector of Maps, one active-map pointer, and one
    shared KeyFrameDatabase across all maps.

KeyFrameDatabase
    Inverted index mapping vocabulary word IDs to KeyFrames.
    Supports fast similarity queries for relocalization, loop closure,
    and map merging.  Uses native DBoW2 (via the `dbow` Python package)
    when available; falls back to a pure-Python L1-score implementation
    for development / CI without native libraries.

Typical usage
-------------
    from slam_pkg.core import Atlas, KeyFrame, MapPoint

    # On startup
    atlas = Atlas(vocabulary_path='/path/to/ORBvoc.bin')
    active_map = atlas.create_new_map()

    # Local Mapping inserts a new KeyFrame
    kf = KeyFrame(pose=T, keypoints=kps, descriptors=desc, timestamp=t)
    bow_vec = atlas.get_keyframe_database().compute_bow_vector(desc)
    kf.bow_vector = bow_vec
    active_map.add_keyframe(kf)
    atlas.get_keyframe_database().add(kf)

    # Local Mapping creates a new MapPoint
    mp = MapPoint(position=xyz, descriptor=d, keyframe=kf, kp_idx=i)
    active_map.add_map_point(mp)

    # Loop Closure queries the database
    candidates = atlas.get_keyframe_database().query(query_bow, n_results=5)

Thread safety
-------------
Every class uses a threading.Lock on all public methods and properties.
Classes also support the context-manager protocol (`with obj: ...`) for
multi-step atomic operations.
"""

from .map_point import MapPoint
from .keyframe import KeyFrame
from .map import Map
from .atlas import Atlas
from .keyframe_database import KeyFrameDatabase

__all__ = [
    "MapPoint",
    "KeyFrame",
    "Map",
    "Atlas",
    "KeyFrameDatabase",
]
