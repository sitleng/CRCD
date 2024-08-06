# Comprehensive Robotic Cholecystectomy Dataset (CRCD)
by Ki-Hwan Oh, Leonardo Borgioli, Alberto Mangano, Valentina Valle, Marco Di Pangrazio, Francesco Toti, Gioia Pozza, Luciano Ambrosini, Alvaro Ducas, Miloš Žefran, Liaohai Chen, Pier Cristoforo Giulianotti

[[arXiv](https://arxiv.org/abs/2312.01183)] [[BibTeX](https://uofi.box.com/s/0cxpk70we719hxcqsdn3bx05lw9yfsth)]

## Dataset

You can download the dataset mentioned in the paper through the following link:

https://uofi.box.com/s/5u2njsggi2qp5og4pg97ncw79anlw1qa

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

## OpenCV CUDA Build
The o


## Tutorials
- There is a tutorial notebook on accessing each recording in the dataset. [CRCD Tutorial](notebooks/CRCD_Tutorial.ipynb)

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
  doi={10.1109/ISMR63436.2024.10585836}}

```
