"""Read-only metric marker observations, independent of encoder forward kinematics."""
import cv2
import numpy as np


MARKER_SIZES_M = {0: .04, 1: .03}


def observe_markers(image, camera_matrix, distortion=None):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 3
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers(image)
    if ids is None:
        return []
    result = []
    k = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    for marker, values in zip(ids.ravel(), corners):
        marker = int(marker)
        if marker not in MARKER_SIZES_M:
            continue
        size = MARKER_SIZES_M[marker]
        # IPPE_SQUARE requires these four points in this exact order.
        object_points = np.array([[-size/2, size/2, 0], [size/2, size/2, 0],
                                  [size/2, -size/2, 0], [-size/2, -size/2, 0]])
        pixels = values.reshape(4, 2).astype(float)
        solved = cv2.solvePnPGeneric(object_points, pixels, k, distortion,
                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        for rotation, translation in zip(solved[1], solved[2]):
            if translation[2, 0] <= 0:
                continue
            rotation, translation = cv2.solvePnPRefineLM(
                object_points, pixels, k, distortion, rotation, translation)
            projected, _ = cv2.projectPoints(object_points, rotation, translation, k, distortion)
            error = np.linalg.norm(projected.reshape(4, 2)-pixels, axis=1)
            candidates.append(dict(rotation_vector=rotation.ravel().tolist(),
                                   translation_m=translation.ravel().tolist(),
                                   reprojection_rms_px=float(np.sqrt(np.mean(error**2)))))
        candidates.sort(key=lambda candidate: candidate['reprojection_rms_px'])
        result.append(dict(marker_id=marker, black_border_side_m=size,
                           corners_px=pixels.tolist(),
                           minimum_side_px=float(np.min(np.linalg.norm(pixels-np.roll(pixels, 1, axis=0), axis=1))),
                           pose_candidates=candidates,
                           encoder_model_used=False, robot_configuration_modified=False))
    return result
