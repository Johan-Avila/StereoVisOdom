#!/usr/bin/python

import rospkg

import os
import cv2
import numpy as np
import yaml
import progressbar

sequence = str(input("secuencia :"))

# configuracionde pwd y files
ros_Pack = rospkg.RosPack()
pwd_src = ros_Pack.get_path('info_stereo_vis_odom')+"/KITTI/"

# base de datos
file_pwd = pwd_src + "config/pwd.yaml"
with open(file_pwd, 'r') as f:
    pwd_dataset = yaml.load(f)["pwd"]["dataset"]

# nombres de imaganes
names_img = sorted(os.listdir(pwd_dataset+sequence+'/image_0/'))

# parametros
sequence_tmp = int(sequence)
file_params = pwd_src + "config/"
if 0 <= sequence_tmp and sequence_tmp <= 2:
    file_params += "params_0_2.yaml"
elif 3 == sequence_tmp:
    file_params += "params_3.yaml"
elif 4 <= sequence_tmp and sequence_tmp <= 10:
    file_params += "params_4_10.yaml"

with open(file_params, 'r') as f:
    params = yaml.load(f)["stereo_vis_odom"]

# configuracion de parametros
mp0 = np.array(
    params["stereoCalibration"]['matrixProjection']['mp0']).reshape((3, 4))
mp1 = np.array(
    params["stereoCalibration"]['matrixProjection']['mp1']).reshape((3, 4))

mp0_k, mp0_r, mp0_t, _, _, _, _ = cv2.decomposeProjectionMatrix(mp0)
mp1_k, mp1_r, mp1_t, _, _, _, _ = cv2.decomposeProjectionMatrix(mp1)

mp0_t = (mp0_t / mp0_t[3])[:3]
mp1_t = (mp1_t / mp1_t[3])[:3]

# parametros mapa de profundidad
distance_cameras = mp1_t[0] - mp0_t[0]
distance_focal = mp0_k[0][0]

# parametros para 3d
cx = mp0_k[0, 2]
cy = mp0_k[1, 2]
fx = mp0_k[0, 0]
fy = mp0_k[1, 1]

sad_window = 6
num_disparities = sad_window * 16
block_size = 11

disparity_StereoSGBM = cv2.StereoSGBM_create(numDisparities=num_disparities,
                                                minDisparity=0,
                                                blockSize=block_size,
                                                P1=8 * 1 * block_size ** 2,
                                                P2=32 * 1 * block_size ** 2,
                                                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

factor_disparity = 0.05
Width = params["stereoCalibration"]["size"]["Width"]
Height = params["stereoCalibration"]["size"]["Height"]

# Detector orb
orb = cv2.ORB_create(nfeatures=1000)

# emparejar esquienas entre poses (Bf)
bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)

# poses
bandera_run = False

# velocidad
vel_mea_k_1 = 0
vel_mea_poses = []
vel_var_k_1 = 0
vel_std_poses = []

# estimate motion
ransac_reprojectionError = 2.0
matrix_pose_tmp = np.eye(4)
matrix_poses = [matrix_pose_tmp]

N = len(names_img)
bar_cont = 0
bar = progressbar.ProgressBar(max_value=N, redirect_stdout=True)

# Recorrer la secuencia
for name_img in names_img:
    # print("img = "+name_img)

    img_left_k = cv2.imread(pwd_dataset+sequence+'/image_0/'+name_img)
    img_right_k = cv2.imread(pwd_dataset+sequence+'/image_1/'+name_img)

    map_disparity_k = disparity_StereoSGBM.compute(
        img_left_k, img_right_k).astype(np.float32)/16.0

    kp_left_k, des_left_k = orb.detectAndCompute(img_left_k, None)

    if bandera_run == False:
        bandera_run = True
        map_disparity_k_1 = map_disparity_k
        kp_left_k_1 = kp_left_k
        des_left_k_1 = des_left_k

        bar_cont = bar_cont+1
        bar.update(bar_cont)
        continue

    disparity_k_1_min = np.amax(map_disparity_k_1)*factor_disparity
    vel = []
    good_match = []
    for match in bf.match(des_left_k_1, des_left_k):
        u_k_1, v_k_1 = kp_left_k_1[match.queryIdx].pt
        u_k, v_k = kp_left_k[match.trainIdx].pt
        disparity_k_1 = map_disparity_k_1[int(
            round(v_k_1)), int(round(u_k_1))]
        if disparity_k_1 < disparity_k_1_min:
            continue
        good_match.append(match)
        vel.append(np.sqrt(np.power(u_k_1-u_k, 2)+np.power(v_k_1-v_k, 2)))

    vel_std_k = np.std(vel)
    vel_mea_k = np.mean(vel)

    if len(vel_std_poses) == 6:
        vel_max_std_poses = np.amax(vel_std_poses)
        if vel_max_std_poses < vel_std_k:
            vel_std_k = (vel_max_std_poses*2)-(np.amin(vel_std_poses))
            vel_mea_x = vel_mea_poses[:5]
            vel_mea_y = vel_mea_poses[1:]
            try:
                reg = np.polyfit(vel_mea_x, vel_mea_y, 1)
                ry_vel_mea_k = np.polyval(reg, vel_mea_y[4])
                if ry_vel_mea_k > 0:
                    vel_mea_k = ry_vel_mea_k
            except:
                pass
        vel_mea_poses.pop(0)
        vel_std_poses.pop(0)

    vel_var_k = vel_std_k**2
    vel_mea_k_pre = vel_mea_k + vel_mea_k_1
    vel_var_k_pre = vel_var_k + vel_var_k_1

    vel_mea_k = ((vel_mea_k_pre*vel_var_k)+(vel_mea_k *
                                            vel_var_k_pre))/(vel_var_k+vel_var_k_pre)
    vel_var_k = (vel_var_k*vel_var_k_pre)/(vel_var_k+vel_var_k_pre)

    vel_max_k = vel_mea_k+((np.sqrt(vel_var_k))*3)

    vel = []
    points_3D_k_1 = []
    pointes_2D_k = []

    for match in good_match:
        u_k_1, v_k_1 = kp_left_k_1[match.queryIdx].pt
        u_k, v_k = kp_left_k[match.trainIdx].pt
        vel_k = np.sqrt(np.power(u_k_1-u_k, 2)+np.power(v_k_1-v_k, 2))

        if vel_max_k < vel_k:
            continue

        disparity_k_1 = map_disparity_k_1[int(
            round(v_k_1)), int(round(u_k_1))]
        z_k_1 = (distance_focal * distance_cameras/disparity_k_1)[0]
        x_k_1 = z_k_1 * (u_k_1 - cx) / fx
        y_k_1 = z_k_1 * (v_k_1 - cy) / fy

        vel.append(vel_k)
        points_3D_k_1.append([x_k_1, y_k_1, z_k_1])
        pointes_2D_k.append([u_k, v_k])

    vel_std_k = np.std(vel)
    vel_mea_k = np.mean(vel)

    vel_std_poses.append(vel_std_k)
    vel_mea_poses.append(vel_mea_k)

    vel_mea_k_1 = vel_mea_k
    vel_var_k_1 = vel_std_k**2

    points_3D_k_1 = np.array(points_3D_k_1, dtype=np.float32)
    pointes_2D_k = np.array(pointes_2D_k, dtype=np.float32)

    _, rvec, tvec, inliers = cv2.solvePnPRansac(
        points_3D_k_1, pointes_2D_k, mp0_k, None,
        reprojectionError=ransac_reprojectionError,
        flags=cv2.SOLVEPNP_P3P)

    matrix_motion = np.eye(4)
    matrix_motion[:3, :3] = cv2.Rodrigues(rvec)[0]
    matrix_motion[:3, 3] = tvec.T
    matrix_pose_tmp = matrix_pose_tmp.dot(np.linalg.inv(matrix_motion))
    matrix_poses.append(matrix_pose_tmp)

    map_disparity_k_1 = map_disparity_k
    kp_left_k_1 = kp_left_k
    des_left_k_1 = des_left_k

    bar_cont = bar_cont+1
    bar.update(bar_cont)

bar.finish()

print("="*20)
print("sequence:", sequence)
print("="*20)
print("guardar ruta")

# matrices de calibracion de las camaras
with open(pwd_src+"route/readme_gt.yaml", 'r') as f:
    readme_gt = yaml.load(f)

file = pwd_src+"route/"+str(sequence)+"_route.txt"
with open(file, "w+") as f:
    for matrix_motion in matrix_poses[0:readme_gt["final_pose_gr_"+str(sequence)]+1]:
        f.write(str(matrix_motion[0][0]) + " ")
        f.write(str(matrix_motion[0][1]) + " ")
        f.write(str(matrix_motion[0][2]) + " ")
        f.write(str(matrix_motion[0][3]) + " ")
        f.write(str(matrix_motion[1][0]) + " ")
        f.write(str(matrix_motion[1][1]) + " ")
        f.write(str(matrix_motion[1][2]) + " ")
        f.write(str(matrix_motion[1][3]) + " ")
        f.write(str(matrix_motion[2][0]) + " ")
        f.write(str(matrix_motion[2][1]) + " ")
        f.write(str(matrix_motion[2][2]) + " ")
        f.write(str(matrix_motion[2][3]) + "\n")
