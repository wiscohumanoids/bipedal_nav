"""
loop_closure -- Thread 3: Loop Closure & Map Merging.

Person 5: PlaceRecognition, PlaceRecognitionResult
Person 6: (correction.py, map_merger.py — added separately)
"""

from .place_recognition import PlaceRecognition, PlaceRecognitionResult

__all__ = [
    "PlaceRecognition",
    "PlaceRecognitionResult",
]
