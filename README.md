# DXSLAM
This is the offline version of dxslam,online version is here!We developer this program base on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)

# 1. Prerequisites
We have tested the library in **Ubuntu 16.04**and **18.04**, but it should be easy to compile in other platforms.
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
Download the [vocabulary file](https://github.com/ivipsourcecode/DX-SLAM/releases/download/1.0.0/DXSLAM.tar.xz) and put it into the ./Vocabulary directory  
Download the [hf-net mode file](https://github.com/ivipsourcecode/DX-SLAM/releases/download/1.0.0/model.tar.xz) and put it into the ./hf-net directory  
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

2. Associate RGB images and depth images using the python script [associate.py]

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```
3. Get HF-net output
  ```
  cd hf-net
  python3 getFeature.py image/path/to/rgb output/featuer/path
  ```
    
4. Execute the following command. Change `IVIP.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.
  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE OUTPUT/FEATURE/PATH
  ```
  
# 4. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM  RGB-D cameras. We use the calibration model of OpenCV. RGB-D input must be synchronized and depth registered.

# 5. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

