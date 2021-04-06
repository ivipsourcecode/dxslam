# DXSLAM

DXSLAM is a visual SLAM system based on deep CNN feature extraction. Please

* clone this repo if you want to run an offline evaluation with e.g. the TUM dataset, or
* clone [dxslam_ros](https://github.com/cedrusx/dxslam_ros) and [deep_features](https://github.com/cedrusx/deep_features) if you want a ROS version to work with a live camera or ROS bags e.g. from the OpenLORIS-Scene datasets, or
* clone [deep_features](https://github.com/cedrusx/deep_features) if you are interested in deep feature extraction only.

Technical details are described in [this paper](https://arxiv.org/pdf/2008.05416) (to be published in IROS 2020):

> Dongjiang Li, Xuesong Shi, Qiwei Long, Shenghui Liu, Wei Yang, Fangshi Wang, Qi Wei, Fei Qiao, "DXSLAM: A Robust and Efficient Visual SLAM System with Deep Features," arXiv preprint arXiv:2008.05416, 2020.

```
@article{li2020dxslam,
  title={{DXSLAM}: A Robust and Efficient Visual {SLAM} System with Deep Features},
  author={Dongjiang Li and Xuesong Shi and Qiwei Long and Shenghui Liu and Wei Yang and Fangshi Wang and Qi Wei and Fei Qiao},
  journal={arXiv preprint arXiv:2008.05416},
  year={2020}
}
```

The SLAM pipeline in this repo is customized from [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).

# 1. Prerequisites
We have tested the library in **Ubuntu 16.04** and **18.04**, but it should be easy to compile in other platforms.
* C++11 or C++0x Compiler
* Pangolin
* OpenCV
* Eigen3
* Dbow、Fbow and g2o (Included in Thirdparty folder)
* tensorflow(1.12)

# 2. Building DXSLAM library and examples
Clone the repository:
```
git clone https://github.com/raulmur/DXSLAM.git DXSLAM
``` 
We provide a script `build.sh` to build the *Thirdparty* libraries and *DXSLAM*. Please make sure you have installed all required dependencies (see section 1). Execute:
```
cd dxslam
chmod +x build.sh
./build.sh
```

This will create **libDXSLAM.so**  at *lib* folder and the executables **rgbd_tum** in *Examples* folder.

# 3. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](https://vision.in.tum.de/lib/exe/fetch.php?tok=5ec47e&media=https%3A%2F%2Fsvncvpr.in.tum.de%2Fcvpr-ros-pkg%2Ftrunk%2Frgbd_benchmark%2Frgbd_benchmark_tools%2Fsrc%2Frgbd_benchmark_tools%2Fassociate.py)

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```
3. Get HF-net output
  ```
  cd hf-net
  python3 getFeature.py image/path/to/rgb output/feature/path
  ```

4. Execute the following command. Change `IVIP.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.
  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/DXSLAM.fbow Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE OUTPUT/FEATURE/PATH
  ```

# 4. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM  RGB-D cameras. We use the calibration model of OpenCV. RGB-D input must be synchronized and depth registered.

# 5. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

