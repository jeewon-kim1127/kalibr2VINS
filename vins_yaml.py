import gc
import numpy as np
import multiprocessing
import sys

import os.path
import yaml

homepath = os.path.expanduser('~')

#modify path and freq
VINS_config_path = homepath +'/vins_ws/src/VINS-Fusion/config/realsense_d435i/' 
kalibr_config_path = homepath +'/kalibr_ws/data/uav2/'
odom_freq = 15 

def saveLRcamParametersYaml(cam_id, resolution, intrinsics, dist_coeffs, VINS_config_path):
    if cam_id == 0: #cam0
        filename = os.path.join(VINS_config_path,'left.yaml')
    elif cam_id == 1: #cam1
        filename = os.path.join(VINS_config_path,'right.yaml')

    data = dict()
    data["model_type"] = 'PINHOLE'
    data["camera_name"] = 'camera'
    data["image_width"] = resolution[0]
    data["image_height"] = resolution[1]

    data["distortion_parameters"] =dict()
    data["distortion_parameters"]["k1"]=dist_coeffs[0]
    data["distortion_parameters"]["k2"]=dist_coeffs[1]
    data["distortion_parameters"]["p1"]=dist_coeffs[2]
    data["distortion_parameters"]["p2"]=dist_coeffs[3]
    data["projection_parameters"] =dict()
    data["projection_parameters"]["fx"]=intrinsics[0]
    data["projection_parameters"]["fy"]=intrinsics[1]
    data["projection_parameters"]["cx"]=intrinsics[2]
    data["projection_parameters"]["cy"]=intrinsics[3]

    with open(filename, 'w') as outfile:
        outfile.write("%YAML:1.0\n\n")
        outfile.write(yaml.dump(data, default_flow_style=None, width=2147483647, sort_keys=True) )
    print("LR write done")

def find_files(filename, search_path):
    for root, dir, files in os.walk(search_path):
        for f in files:
            if filename in f:
                filepath = os.path.join(search_path, f)
                return filepath

def addstring(filename, string, VINS_config_path):
    newfilename = os.path.join(VINS_config_path,'realsense_stereo_imu_config.yaml')
    with open(filename, 'r') as IN, open(newfilename, 'w') as OUT:
        i=0
        for line in IN:
            if "body_T_cam" in line and " !!" not in line:
                newline = line.strip('\n')+string+'\n'
                OUT.write(newline)
                i = i+1
            else:
                OUT.write(line)
    os.remove(filename)
    return newfilename

def saveStereoIMUConfigYaml(kalibr_config_path, VINS_config_path):
    n_cam = 2
    n_imu=1
    print("reading from " +kalibr_config_path)

    camyaml = find_files("-camchain-imucam.yaml", kalibr_config_path)
    with open(camyaml, 'r') as fp:
        camdata = yaml.load(fp, Loader=yaml.FullLoader)
        T_ic = np.zeros((n_cam, 16))
        cam_topic = ['' for x in range(n_cam)] #np.zeros((n_cam, 1))
        for cam_id in range(0,n_cam):
            cam_name = 'cam'+str(cam_id)
            cam = camdata.get(cam_name)
            dist_coeffs = cam.get('distortion_coeffs')
            intrinsics = cam.get('intrinsics')
            resolution = cam.get('resolution')
            cam_topic[cam_id] = cam.get('rostopic')

            saveLRcamParametersYaml(cam_id, resolution, intrinsics, dist_coeffs, VINS_config_path)
            T_ci = np.array(cam.get('T_cam_imu'))
            T_ic_arr = np.linalg.inv(T_ci)
            T_ic[cam_id,:] = T_ic_arr.flatten().tolist()


    imuyaml = find_files("-imu.yaml", kalibr_config_path)
    with open(imuyaml, 'r') as fp:
        imudata = yaml.load(fp, Loader=yaml.FullLoader)
        imu = imudata.get('imu0')
        an = imu.get('accelerometer_noise_density')
        aw = imu.get('accelerometer_random_walk')
        gn = imu.get('gyroscope_noise_density')
        gw = imu.get('gyroscope_random_walk')
        imu_topic = imu.get('rostopic')

    filename = os.path.join(VINS_config_path,'realsense_stereo_config_imu_tmp.yaml')
    outfile = open(filename, 'w')
    outfile.write("%YAML:1.0\n\n")
    data = dict()

    data["imu"] = n_imu
    data["num_of_cam"] = n_cam

    data["imu_topic"] = imu_topic
    data["image0_topic"] = cam_topic[0]
    data["image1_topic"] = cam_topic[1]
    data["output_path"] = homepath+'/output/' #modify if needed!

    data["cam0_calib"] = "left.yaml"
    data["cam1_calib"] = "right.yaml"
    data["image_width"] = resolution[0]
    data["image_height"] = resolution[1]

    data["estimate_extrinsic"] = 1
    b2c0_arr = [ float(val) for val in  T_ic[0,:]]
    data["body_T_cam0"] = dict()
    data["body_T_cam0"]["rows"] = 4
    data["body_T_cam0"]["cols"] = 4
    data["body_T_cam0"]["dt"] = 'd'
    data["body_T_cam0"]["data"] = b2c0_arr

    b2c1_arr = [ float(val) for val in T_ic[1,:]]
    data["body_T_cam1"] = dict()
    data["body_T_cam1"]["rows"] = 4
    data["body_T_cam1"]["cols"] = 4
    data["body_T_cam1"]["dt"] = 'd'
    data["body_T_cam1"]["data"] = b2c1_arr

    data["multiple_thread"] = 1
    data["max_cnt"] = 150
    data["min_dist"] = 30
    data["freq"] = odom_freq
    data["F_threshold"] = 1.0
    data["show_track"] = 0
    data["flow_back"] = 1

    data["max_solver_time"] = 0.04
    data["max_num_iterations"] = 8
    data["keyframe_parallax"] = 10.0

    data["acc_n"] = an
    data["acc_w"] = aw
    data["gyr_n"] = gn
    data["gyr_w"] = gw
    data["g_norm"] = 9.805 

    data["estimate_td"] = 1
    data["td"] = 0.00   

    data["load_previous_pose_graph"] = 0
    data["pose_graph_save_path"] = data["output_path"]+'/pose_graph/' 
    data["save_image"] = 0

    outfile.write( yaml.dump(data, default_flow_style=None, width=2147483647,sort_keys=True) )
    outfile.close()
    newfilename = addstring(filename, " !!opencv-matrix", VINS_config_path)
    print("\nstereo-imu config yaml file written to "+newfilename)

if __name__ == "__main__":
    saveStereoIMUConfigYaml(kalibr_config_path, VINS_config_path)
