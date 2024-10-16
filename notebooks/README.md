# Base Tutorials

## Intro

Here are some notebooks to guide for each usage:

- [Basic Tutorial 1](crcd_base.ipynb): Shows how to load the stereo endoscopic images, kinematic data, and the pedal data.

- [Basic Tutorial 2](crcd_dataframe.ipynb): Shows how to create a pandas dataframe with synchronized robot arm kinematics and the console pedal inputs.

The following setups are required to run the base tutorials.

## ROS Setup

The kinematic and pedal datasets are recorded in rosbag files. 

The files have been tested without errors on ROS1 Noetic, installed in an Ubuntu 20.04 device.

- First, install ROS Noetic following the instructions from the official website:

  http://wiki.ros.org/noetic/Installation/Ubuntu
  
- Create the ROS workspace:
  ```
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  ```
- Clone our repository
  ```
  git clone https://github.com/Borgioli/crcd_ros.git
  ```
- Build the package
  ```
  cd ..
  catkin build --summary
  ```

## Tesseract OCR

For real-time Tesseract OCR in Python, it is recommended that you install tesserocr (https://github.com/sirfz/tesserocr).

In Ubuntu, you might need to source the ```TESSDATA_PREFIX``` to the tessdata directory.

- Open the bashrc with your preferred text editor (sublime text is used in this example).
```
sudo subl ~/.bashrc
```
- Add this line at the end of the file. The path can be different.
```
export TESSDATA_PREFIX=/usr/share/tessdata
```
- Then source bashrc.
```
source ~/.bashrc
```

## Pandas

Converting the rosbag recordings to pandas dataframes are useful when loading the robot arm kinematics and console pedal inputs and use them to train different types of machine learning models. 

Please follow these steps from the official website (https://pandas.pydata.org/docs/getting_started/install.html).

## Citation

Please cite our paper if you used our dataset in any form: 
```
@INPROCEEDINGS{koh2024crcd,
  author={Oh, Ki-Hwan and Borgioli, Leonardo and Mangano, Alberto and Valle, Valentina and Di Pangrazio, Marco and Toti, Francesco and Pozza, Gioia and Ambrosini, Luciano and Ducas, Alvaro and Žefran, Miloš and Chen, Liaohai and Giulianotti, Pier Cristoforo},
  booktitle={2024 International Symposium on Medical Robotics (ISMR)}, 
  title={Comprehensive Robotic Cholecystectomy Dataset (CRCD): Integrating Kinematics, Pedal Signals, and Endoscopic Videos}, 
  year={2024},
  volume={},
  number={},
  pages={1-7},
  keywords={Medical robotics;Automation;Robot vision systems;Liver;Kinematics;Predictive models;Cameras},
  doi={10.1109/ISMR63436.2024.10585836}
}

```
