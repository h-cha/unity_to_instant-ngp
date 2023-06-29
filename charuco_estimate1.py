import numpy as np
import cv2
import os
import json
from cv2 import aruco


def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()


def sharpness(imagePath):
    image = cv2.imread(imagePath)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    return fm


def closest_point_2_lines(oa, da, ob, db):
    # returns point closest to both rays of form o+t*d, and a weight factor that goes to 0 if the lines are parallel
    da = da / np.linalg.norm(da)
    db = db / np.linalg.norm(db)
    c = np.cross(da, db)
    denom = np.linalg.norm(c)**2
    t = ob - oa
    ta = np.linalg.det([t, db, c]) / (denom + 1e-10)
    tb = np.linalg.det([t, da, c]) / (denom + 1e-10)
    if ta > 0:
        ta = 0
    if tb > 0:
        tb = 0
    return (oa+ta*da+ob+tb*db) * 0.5, denom


def estimateCamPoseFromBoard(frame, camMatrix, distCoeffs):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        gray, aruco_dict)

    if ids is None:
        print("cannot detect markers.")
        return None, None, None

    criteria = (cv2.TERM_CRITERIA_EPS +
                cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    for corner in corners:
        cv2.cornerSubPix(gray, corner, winSize=(
            3, 3), zeroZone=(-1, -1), criteria=criteria)

    size_of_marker = 0.0171  # side length of the marker in meter

    rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(
        corners, size_of_marker, camMatrix, distCoeffs)

    imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    for i in range(len(tvecs)):
        imaxis = cv2.drawFrameAxes(
            imaxis, camMatrix, distCoeffs, rvecs[i], tvecs[i], 0.01)

    response, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board, cameraMatrix=camMatrix, distCoeffs=distCoeffs)

    if response > 6:
        ret, p_rvec, p_tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board, camMatrix, distCoeffs,
            np.empty(1), np.empty(1))
        if ret:
            cv2.drawFrameAxes(imaxis, camMatrix, distCoeffs,
                              p_rvec, p_tvec, 0.1)
            rotmat, _ = cv2.Rodrigues(p_rvec)

            w_rotmat = rotmat.T
            w_tvec = -(rotmat.T).dot(p_tvec)

            imaxis = cv2.resize(imaxis, dsize=None, fx=0.5, fy=0.5)

            return w_rotmat, w_tvec, imaxis
    else:
        print("responce is below threashold")

    return None, None, None


def main():
    data_dir = "./data/230613_remocon"
    png_dir = os.path.join(data_dir, "png1/")
    json_path = os.path.join(data_dir, "calib_result.json")
    res_path = os.path.join(data_dir, "transforms_charuco1.json")

    # data_dir = "./data/220723_teacup/png/min/"
    # json_path = "./calib_result.json"

    # data_dir = "./data/220724_teacup2/png/"
    # json_path = "./calib_result2.json"

    images = np.array(
        [png_dir + f for f in os.listdir(png_dir) if f.endswith(".JPG")])
    order = np.argsort(
        [int(p.split("/")[-1].split(".")[0]) for p in images])
    # order = np.argsort(
    #     [int(p.split("/")[-1].split(".")[0].split("_")[1]) for p in images])

    images = images[order]
    print(images)

    with open(json_path) as f:
        calib_params = json.load(f)

    mtx = np.array(calib_params["mtx"])
    dist = np.array(calib_params["dist"]).T

    frame = cv2.imread(images[0])
    print(frame.shape)
    transform_dict = {}
    transform_dict["fl_x"] = mtx[0, 0]
    transform_dict["fl_y"] = mtx[1, 1]

    transform_dict["k1"] = dist[0][0]
    transform_dict["k2"] = dist[1][0]
    transform_dict["p1"] = dist[2][0]
    transform_dict["p2"] = dist[3][0]
    transform_dict["cx"] = mtx[0, 2]
    transform_dict["cy"] = mtx[1, 2]
    transform_dict["w"] = frame.shape[1]
    transform_dict["h"] = frame.shape[0]
    transform_dict["aabb_scale"] = 2

    print(transform_dict)

    transform_dict["frames"] = []

    for idx, im in enumerate(images):
        frame_dict = {}
        frame_dict["file_path"] = im
        frame_dict["sharpness"] = sharpness(im)
        print(frame_dict["sharpness"])

        frame = cv2.imread(im)
        if frame_dict["sharpness"] < 120:
            frame = cv2.resize(frame, dsize=None, fx=0.5, fy=0.5)
            cv2.imshow("not sharp", frame)
            cv2.waitKey(10)
            continue

        w_rotmat, w_tvec, imaxis = estimateCamPoseFromBoard(frame, mtx, dist)
        if w_rotmat is None:
            continue

        w_rt = np.zeros((4, 4))
        w_rt[0:3, 0:3] = w_rotmat
        w_rt[0:3, 3] = w_tvec.flatten()
        w_rt[3, 3] = 1
        frame_dict["transform_matrix"] = w_rt

        transform_dict["frames"].append(frame_dict)

        cv2.imshow("test", imaxis)
        cv2.waitKey(10)

    print("flip coordinates ...")
    for f in transform_dict["frames"]:
        c2w = f["transform_matrix"]
        c2w[0:3, 2] *= -1   # flip the y and z axis
        c2w[0:3, 1] *= -1
        c2w = c2w[[1, 0, 2, 3], :]  # swap y and z
        c2w[2, :] *= -1  # flip whole world upside down

    # print("computing center of attention...")
    # totw = 0.0
    # totp = np.array([0.0, 0.0, 0.0])

    # count = 0
    # for f in transform_dict["frames"]:
    #     mf = f["transform_matrix"][0:3, :]
    #     for g in transform_dict["frames"]:
    #         mg = g["transform_matrix"][0:3, :]
    #         p, w = closest_point_2_lines(
    #             mf[:, 3], mf[:, 2], mg[:, 3], mg[:, 2])
    #         if w > 0.01:
    #             totp += p*w
    #             totw += w
    #         count += 1
    # print("totw:", totw, ", cnt:", count, ", avgw:", totw/count)

    # totp /= totw
    # transform_dict["totp"] = totp.tolist()
    # print(totp)  # the cameras are looking at totp

    # for f in transform_dict["frames"]:
    #     f["transform_matrix"][0:3, 3] -= totp

    # avglen = 0.
    # for f in transform_dict["frames"]:
    #     avglen += np.linalg.norm(f["transform_matrix"][0:3, 3])
    # avglen /= len(transform_dict["frames"])

    # print("avg camera distance from origin", avglen)
    # print("scale: ", 4.0 / avglen)
    # transform_dict["totw"] = 4.0 / avglen

    # for f in transform_dict["frames"]:
    #     f["transform_matrix"][0:3, 3] *= 4.0 / avglen

    for f in transform_dict["frames"]:
        f["transform_matrix"] = f["transform_matrix"].tolist()

    with open(res_path, "w") as fp:
        json.dump(transform_dict, fp, indent=2)


if __name__ == "__main__":
    workdir = "./workdir/"
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    # 28.5mm x 17.1mm
    board = aruco.CharucoBoard_create(10, 7, 0.0285, 0.0171, aruco_dict)
    imboard = board.draw((1200, 1000), 10, 1)
    cv2.imwrite(workdir + "chessboard.tiff", imboard)

    main()
