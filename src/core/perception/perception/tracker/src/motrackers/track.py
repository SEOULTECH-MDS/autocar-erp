import numpy as np
from perception.tracker.src.motrackers.kalman_tracker import KFTracker2D


class Track:
    """
    Track containing attributes to track various objects.

    Args:
        frame_id (int): Camera frame id.
        track_id (int): Track Id
        center (numpy.ndarray): Center point as (x_c,y_c) of the track.
        detection_confidence (float): Detection confidence of the object (probability).
        class_id (str or int): Class label id.
        lost (int): Number of times the object or track was not tracked by tracker in consecutive frames.
        iou_score (float): Intersection over union score.
        data_output_format (str): Output format for data in tracker.
            Options include ``['mot_challenge', 'visdrone_challenge']``. Default is ``mot_challenge``.
        kwargs (dict): Additional key word arguments.

    """

    count = 0

    metadata = dict(
        data_output_formats=['mot_challenge', 'visdrone_challenge']
    )

    def __init__(
        self,
        track_id,
        frame_id,
        center,
        detection_confidence,
        class_id=None,
        lost=0,
        iou_score=0.,
        data_output_format='mot_challenge',
        **kwargs
    ):
        assert data_output_format in Track.metadata['data_output_formats']
        Track.count += 1
        self.id = track_id

        self.detection_confidence_max = 0.
        self.lost = 0
        self.age = 0

        self.update(frame_id, center, detection_confidence, class_id=class_id, lost=lost, iou_score=iou_score, **kwargs)

        if data_output_format == 'mot_challenge':
            self.output = self.get_mot_challenge_format
        elif data_output_format == 'visdrone_challenge':
            self.output = self.get_vis_drone_format
        else:
            raise NotImplementedError

    def update(self, frame_id, center, detection_confidence, class_id=None, lost=0, iou_score=0., **kwargs):
        """
        Update the track.

        Args:
            frame_id (int): Camera frame id.
            center (numpy.ndarray): Center point as (x_c,y_c) of the track.
            detection_confidence (float): Detection confidence of the object (probability).
            class_id (int or str): Class label id.
            lost (int): Number of times the object or track was not tracked by tracker in consecutive frames.
            iou_score (float): Intersection over union score.
            kwargs (dict): Additional key word arguments.
        """
        self.class_id = class_id
        self.center = np.array(center)
        self.detection_confidence = detection_confidence
        self.frame_id = frame_id
        self.iou_score = iou_score

        if lost == 0:
            self.lost = 0
        else:
            self.lost += lost

        for k, v in kwargs.items():
            setattr(self, k, v)

        self.detection_confidence_max = max(self.detection_confidence_max, detection_confidence)

        self.age += 1

    @property
    def centroid(self):
        """
        Return the centroid of the bounding box.

        Returns:
            numpy.ndarray: Centroid (x, y) of bounding box.

        """
        return np.array((self.center[0], self.center[1]))

    def get_mot_challenge_format(self):
        """
        Get the tracker data in MOT challenge format as a tuple of elements containing
        `(frame, id, bb_left, bb_top, bb_width, bb_height, conf, x, y, z)`

        References:
            - Website : https://motchallenge.net/

        Returns:
            tuple: Tuple of 10 elements representing `(frame, id, bb_left, bb_top, bb_width, bb_height, conf, x, y, z)`.

        """
        mot_tuple = (
            self.frame_id, self.id, self.center[0], self.center[1], self.detection_confidence,
            -1, -1, -1
        )
        return mot_tuple

    def get_vis_drone_format(self):
        """
        Track data output in VISDRONE Challenge format with tuple as
        `(frame_index, target_id, center_left, center_top, center_width, center_height, score, object_category,
        truncation, occlusion)`.

        References:
            - Website : http://aiskyeye.com/
            - Paper : https://arxiv.org/abs/2001.06303
            - GitHub : https://github.com/VisDrone/VisDrone2018-MOT-toolkit
            - GitHub : https://github.com/VisDrone/

        Returns:
            tuple: Tuple containing the elements as `(frame_index, target_id, center_left, center_top, center_width, center_height,
            score, object_category, truncation, occlusion)`.
        """
        mot_tuple = (
            self.frame_id, self.id, self.center[0], self.center[1],
            self.detection_confidence, self.class_id, -1, -1
        )
        return mot_tuple

    def predict(self):
        """
        Implement to prediction the next estimate of track.
        """
        raise NotImplemented

    @staticmethod
    def print_all_track_output_formats():
        print(Track.metadata['data_output_formats'])

class KFTrackCentroid(Track):
    """
    Track based on Kalman filter used for Centroid Tracking of bounding box in MOT.

    Args:
        track_id (int): Track Id
        frame_id (int): Camera frame id.
        center (numpy.ndarray): Bounding box pixel coordinates as (xmin, ymin, width, height) of the track.
        detection_confidence (float): Detection confidence of the object (probability).
        class_id (str or int): Class label id.
        lost (int): Number of times the object or track was not tracked by tracker in consecutive frames.
        iou_score (float): Intersection over union score.
        data_output_format (str): Output format for data in tracker.
            Options ``['mot_challenge', 'visdrone_challenge']``. Default is ``mot_challenge``.
        process_noise_scale (float): Process noise covariance scale or covariance magnitude as scalar value.
        measurement_noise_scale (float): Measurement noise covariance scale or covariance magnitude as scalar value.
        kwargs (dict): Additional key word arguments.
    """
    def __init__(self, track_id, frame_id, center, detection_confidence, class_id=None, lost=0, iou_score=0.,
                 data_output_format='mot_challenge', process_noise_scale=1.0, measurement_noise_scale=1.0, **kwargs):
        c = np.array((center[0], center[1]))
        self.kf = KFTracker2D(c, process_noise_scale=process_noise_scale, measurement_noise_scale=measurement_noise_scale)
        super().__init__(track_id, frame_id, center, detection_confidence, class_id=class_id, lost=lost,
                         iou_score=iou_score, data_output_format=data_output_format, **kwargs)

    def predict(self):
        """
        Predicts the next estimate of the bounding box of the track.

        Returns:
            numpy.ndarray: Center point as (x_c,y_c) of the track.

        """
        s = self.kf.predict()
        xmid, ymid = s[0], s[3]
        return np.array([xmid, ymid])

    def update(self, frame_id, center, detection_confidence, class_id=None, lost=0, iou_score=0., **kwargs):
        super().update(
            frame_id, center, detection_confidence, class_id=class_id, lost=lost, iou_score=iou_score, **kwargs)
        self.kf.update(self.centroid)
