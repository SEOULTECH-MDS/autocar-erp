from collections import OrderedDict
import numpy as np
from scipy.spatial import distance
from scipy.optimize import linear_sum_assignment
from perception.tracker.src.motrackers.tracker import Tracker
from perception.tracker.src.motrackers.track import KFTrackCentroid
from perception.tracker.src.motrackers.utils.misc import get_centroid


def assign_tracks2detection_centroid_distances(center_tracks, center_detections, distance_threshold=10.):
    """
    Assigns detected bounding boxes to tracked bounding boxes using IoU as a distance metric.

    Args:
        center_tracks (numpy.ndarray): Tracked bounding boxes with shape `(n, 4)`
            and each row as `(xmin, ymin, width, height)`.
        center_detections (numpy.ndarray): detection bounding boxes with shape `(m, 4)` and
            each row as `(xmin, ymin, width, height)`.
        distance_threshold (float): Minimum distance between the tracked object
            and new detection to consider for assignment.

    Returns:
        tuple: Tuple containing the following elements:
            - matches (numpy.ndarray): Array of shape `(n, 2)` where `n` is number of pairs formed after matching tracks to detections. This is an array of tuples with each element as matched pair of indices`(track_index, detection_index)`.
            - unmatched_detections (numpy.ndarray): Array of shape `(m,)` where `m` is number of unmatched detections.
            - unmatched_tracks (numpy.ndarray): Array of shape `(k,)` where `k` is the number of unmatched tracks.

    """

    if (center_tracks.size == 0) or (center_detections.size == 0):
        return np.empty((0, 2), dtype=int), np.arange(len(center_detections), dtype=int), np.empty((0,), dtype=int)

    if len(center_tracks.shape) == 1:
        center_tracks = center_tracks[None, :]

    if len(center_detections.shape) == 1:
        center_detections = center_detections[None, :]

    estimated_track_centroids = center_tracks
    detection_centroids = center_detections
    centroid_distances = distance.cdist(estimated_track_centroids, detection_centroids)

    assigned_tracks, assigned_detections = linear_sum_assignment(centroid_distances)

    unmatched_detections, unmatched_tracks = [], []

    for d in range(center_detections.shape[0]):
        if d not in assigned_detections:
            unmatched_detections.append(d)

    for t in range(center_tracks.shape[0]):
        if t not in assigned_tracks:
            unmatched_tracks.append(t)

    # filter out matched with high distance between centroids
    matches = []
    for t, d in zip(assigned_tracks, assigned_detections):
        if centroid_distances[t, d] > distance_threshold:
            unmatched_detections.append(d)
            unmatched_tracks.append(t)
        else:
            matches.append((t, d))

    if len(matches):
        matches = np.array(matches)
    else:
        matches = np.empty((0, 2), dtype=int)

    return matches, np.array(unmatched_detections), np.array(unmatched_tracks)


class CentroidKF_Tracker(Tracker):
    """
    Kalman filter based tracking of multiple detected objects.

    Args:
        max_lost (int): Maximum number of consecutive frames object was not detected.
        tracker_output_format (str): Output format of the tracker.
        process_noise_scale (float or numpy.ndarray): Process noise covariance matrix of shape (3, 3) or
            covariance magnitude as scalar value.
        measurement_noise_scale (float or numpy.ndarray): Measurement noise covariance matrix of shape (1,)
            or covariance magnitude as scalar value.
        time_step (int or float): Time step for Kalman Filter.
    """

    def __init__(
            self,
            max_lost=1,
            centroid_distance_threshold=0.1,
            tracker_output_format='mot_challenge',
            process_noise_scale=1.0,
            measurement_noise_scale=1.0,
            time_step=1
    ):
        self.time_step = time_step
        self.process_noise_scale = process_noise_scale
        self.measurement_noise_scale = measurement_noise_scale
        self.centroid_distance_threshold = centroid_distance_threshold
        self.kalman_trackers = OrderedDict()
        super().__init__(max_lost, tracker_output_format)

    def _add_track(self, frame_id, center, detection_confidence, class_id, **kwargs):
        self.tracks[self.next_track_id] = KFTrackCentroid(
            self.next_track_id, frame_id, center, detection_confidence, class_id=class_id,
            data_output_format=self.tracker_output_format, process_noise_scale=self.process_noise_scale,
            measurement_noise_scale=self.measurement_noise_scale, **kwargs
        )
        self.next_track_id += 1

    def update(self, centers, detection_scores, class_ids):
        self.frame_count += 1
        center_detections = np.array(centers, dtype='int')

        track_ids = list(self.tracks.keys())
        center_tracks = []
        for track_id in track_ids:
            center_tracks.append(self.tracks[track_id].predict())
        center_tracks = np.array(center_tracks)

        if len(centers) == 0:
            for i in range(len(center_tracks)):
                track_id = track_ids[i]
                center = center_tracks[i, :]
                confidence = self.tracks[track_id].detection_confidence
                cid = self.tracks[track_id].class_id
                self._update_track(track_id, self.frame_count, center, detection_confidence=confidence, class_id=cid, lost=1)
                if self.tracks[track_id].lost > self.max_lost:
                    self._remove_track(track_id)
        else:
            matches, unmatched_detections, unmatched_tracks = assign_tracks2detection_centroid_distances(
                center_tracks, center_detections, distance_threshold=self.centroid_distance_threshold
            )

            for i in range(matches.shape[0]):
                t, d = matches[i, :]
                track_id = track_ids[t]
                center = centers[d, :]
                cid = class_ids[d]
                confidence = detection_scores[d]
                self._update_track(track_id, self.frame_count, center, confidence, cid, lost=0)

            for d in unmatched_detections:
                center = centers[d, :]
                cid = class_ids[d]
                confidence = detection_scores[d]
                self._add_track(self.frame_count, center, confidence, cid)

            for t in unmatched_tracks:
                track_id = track_ids[t]
                center = center_tracks[t, :]
                confidence = self.tracks[track_id].detection_confidence
                cid = self.tracks[track_id].class_id
                self._update_track(track_id, self.frame_count, center, confidence, cid, lost=1)

                if self.tracks[track_id].lost > self.max_lost:
                    self._remove_track(track_id)

        outputs = self._get_tracks(self.tracks)
        return outputs