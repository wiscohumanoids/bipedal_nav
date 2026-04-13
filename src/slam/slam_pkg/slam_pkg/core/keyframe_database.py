"""
KeyFrameDatabase — inverted-index over BoW word IDs for fast place recognition.

In ORB-SLAM3 this wraps DBoW2's Database class, which maintains an
inverted index: word_id → [KeyFrame, ...].

This implementation supports two backends:

  Native backend  (preferred)
      Uses the `dbow` Python package (pip install dbow), which wraps the
      DBoW2/DBoW3 C++ library via pybind11.  Requires a pre-trained vocabulary
      file (.txt or .bin) to be passed as `vocabulary_path`.

  Python-fallback backend  (development / CI without native libs)
      A pure-Python inverted index with L1-score similarity search.
      Implements the same scoring formula as DBoW2 so results are comparable.
      No vocabulary file required.

Public interface (identical in both backends)
---------------------------------------------
  add(kf)                            — index a KeyFrame's bow_vector
  erase(kf)                          — remove a KeyFrame from the index
  query(bow_vector, n, min_score)    — return [(KeyFrame, score), ...] sorted by score

Usage
-----
  # with native DBoW2:
  db = KeyFrameDatabase('/path/to/ORBvoc.bin')

  # pure-Python fallback (testing):
  db = KeyFrameDatabase()

  db.add(kf)                         # called by Local Mapping on KF insertion
  db.erase(kf)                       # called by Local Mapping on KF culling
  results = db.query(kf.bow_vector)  # called by Loop Closure / Relocalization
"""

import threading
import math
from collections import defaultdict

# ---- Try native DBoW2/DBoW3 Python binding ----------------------------
try:
    import dbow  # pip install dbow  (pybind11 wrapper around DBoW2/DBoW3)
    _DBOW_AVAILABLE = True
except ImportError:
    _DBOW_AVAILABLE = False


class KeyFrameDatabase:
    """
    Inverted index over BoW word IDs mapping each word to the KeyFrames
    that contain it.  Used by:
      • Relocalization   (Thread 1 — Tracking)
      • Loop detection   (Thread 3 — Loop Closure)
      • Map merging      (Thread 3 — Map Merging)

    Parameters
    ----------
    vocabulary_path : str or None
        Path to a pre-trained ORB vocabulary (.txt or .bin).
        Required for the native DBoW2 backend; ignored in fallback mode.
    """

    def __init__(self, vocabulary_path: str = None) -> None:
        self._lock: threading.Lock = threading.Lock()

        if _DBOW_AVAILABLE and vocabulary_path is not None:
            # ---- Native DBoW2 backend ----
            self._vocab = dbow.Vocabulary(vocabulary_path)
            self._db = dbow.Database(self._vocab)
            # Maps KeyFrame objects → DBoW2 entry IDs so we can call erase()
            self._kf_to_entry_id: dict = {}
            self._backend: str = "dbow"
        else:
            # ---- Pure-Python fallback ----
            # inverted_index[word_id] = list of KeyFrame objects
            self._inverted_index: dict = defaultdict(list)
            self._backend: str = "python"

        self._vocabulary_path = vocabulary_path

    # ==================================================================
    # Add / Erase
    # ==================================================================
    def add(self, keyframe) -> None:
        """
        Index `keyframe` using its precomputed BoW vector.

        The keyframe's bow_vector must already be filled in before calling
        this method (done by Local Mapping after DBoW2 vocabulary transform).
        """
        with self._lock:
            if self._backend == "dbow":
                # dbow.Database.add() returns an integer entry ID
                entry_id = self._db.add(keyframe.bow_vector)
                self._kf_to_entry_id[keyframe] = entry_id
            else:
                # Insert kf into each word's bucket (skip duplicates)
                for word_id in keyframe.bow_vector:
                    bucket = self._inverted_index[word_id]
                    if keyframe not in bucket:
                        bucket.append(keyframe)

    def erase(self, keyframe) -> None:
        """
        Remove `keyframe` from the index.
        Called by Local Mapping when a KeyFrame is culled.
        """
        with self._lock:
            if self._backend == "dbow":
                entry_id = self._kf_to_entry_id.pop(keyframe, None)
                if entry_id is not None:
                    self._db.erase(entry_id)
            else:
                for word_id in keyframe.bow_vector:
                    bucket = self._inverted_index[word_id]
                    try:
                        bucket.remove(keyframe)
                    except ValueError:
                        pass

    # ==================================================================
    # Query
    # ==================================================================
    def query(
        self,
        bow_vector: dict,
        n_results: int = 5,
        min_score: float = 0.0,
    ) -> list:
        """
        Find KeyFrames most similar to `bow_vector`.

        Parameters
        ----------
        bow_vector : dict {word_id(int): weight(float)}
            The BoW vector of the query frame (e.g., the current live frame).
        n_results  : int
            Maximum number of candidates to return.
        min_score  : float  [0, 1]
            Discard candidates with similarity score below this threshold.

        Returns
        -------
        list of (KeyFrame, score) sorted by descending score.
        """
        with self._lock:
            if self._backend == "dbow":
                raw = self._db.query(bow_vector, max_results=n_results)
                return [
                    (r.kf, r.score)
                    for r in raw
                    if r.score >= min_score
                ]
            else:
                return self._python_query(bow_vector, n_results, min_score)

    # ------------------------------------------------------------------
    # Pure-Python L1-score similarity (same formula as DBoW2)
    # ------------------------------------------------------------------
    def _python_query(
        self,
        bow_vector: dict,
        n_results: int,
        min_score: float,
    ) -> list:
        """
        DBoW2 L1 scoring formula::

            score(A, B) = 1 - 0.5 * Σ |a_i - b_i|

        where A and B are already L1-normalised BoW vectors.

        Implementation
        --------------
        1. Build candidate set from the inverted index (only KFs sharing at
           least one word with the query).
        2. Compute L1 score for each candidate.
        3. Return top-N above min_score, sorted descending.
        """
        # Step 1 — gather candidates via inverted index
        candidate_score: dict = defaultdict(float)
        for word_id, weight in bow_vector.items():
            for kf in self._inverted_index.get(word_id, []):
                if not kf.is_bad:
                    # Accumulate common weight (used only for fast pre-filtering)
                    candidate_score[kf] += weight

        if not candidate_score:
            return []

        # Step 2 — compute full L1 score for each candidate
        scored = []
        for kf in candidate_score:
            kf_bv = kf.bow_vector
            # Union of all word IDs in both vectors
            all_words = set(bow_vector.keys()) | set(kf_bv.keys())
            l1_diff = 0.0
            for wid in all_words:
                l1_diff += abs(bow_vector.get(wid, 0.0) - kf_bv.get(wid, 0.0))
            score = 1.0 - 0.5 * l1_diff
            if score >= min_score:
                scored.append((kf, score))

        # Step 3 — sort descending, truncate
        scored.sort(key=lambda x: x[1], reverse=True)
        return scored[:n_results]

    # ==================================================================
    # Vocabulary helpers  (native backend only)
    # ==================================================================
    def compute_bow_vector(self, descriptors) -> dict:
        """
        Convert a descriptor matrix (N×32 uint8 numpy array) into a BoW vector.

        Only available in native DBoW2 mode.  In fallback mode, the caller
        must supply bow vectors externally (e.g., computed via a custom
        vocabulary implementation or left empty for testing).

        Parameters
        ----------
        descriptors : np.ndarray shape (N, 32) uint8

        Returns
        -------
        dict {word_id: weight}
        """
        if self._backend != "dbow":
            # Fallback: return an empty BoW vector
            # Thread teams can replace this with their own vocabulary transform.
            return {}
        with self._lock:
            return self._vocab.transform(descriptors)

    # ==================================================================
    # Introspection
    # ==================================================================
    @property
    def backend(self) -> str:
        """'dbow' if native DBoW2 is available, else 'python'."""
        return self._backend

    def num_entries(self) -> int:
        """Number of KeyFrames currently indexed."""
        with self._lock:
            if self._backend == "dbow":
                return len(self._kf_to_entry_id)
            else:
                # Count unique KFs across all buckets
                seen = set()
                for bucket in self._inverted_index.values():
                    seen.update(bucket)
                return len(seen)

    def __len__(self) -> int:
        return self.num_entries()

    def __repr__(self) -> str:
        return (
            f"KeyFrameDatabase("
            f"backend='{self._backend}', "
            f"entries={self.num_entries()}, "
            f"vocab='{self._vocabulary_path}')"
        )
