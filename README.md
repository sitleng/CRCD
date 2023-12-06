# Comprehensive Robotic Cholecystectomy Dataset (CRCD)
by Ki-Hwan Oh, Leonardo Borgioli, Alberto Mangano, Valentina Valle, Marco Di Pangrazio, Francesco Toti, Gioia Pozza, Luciano Ambrosini, Alvaro Ducas, Miloš Žefran, Liaohai Chen, Pier Cristoforo Giulianotti

[[arXiv](https://arxiv.org/abs/2312.01183)] [[BibTeX](https://uofi.box.com/s/0cxpk70we719hxcqsdn3bx05lw9yfsth)]

## Dataset

You can download the dataset mentioned in the paper through the following link:

https://uofi.box.com/s/5u2njsggi2qp5og4pg97ncw79anlw1qa

## ROS Setup

The kinematic and pedal datasets are recorded in rosbag files. 

The files have been tested with no errors on ROS1 Noetic, installed in an Ubuntu 20.04 device.

- First, install ROS Noetic following the instructions from the official website: http://wiki.ros.org/noetic/Installation/Ubuntu
- Create the ROS workspace:
  ```
  mkdir -p ~/workspace_name_ws/src
  cd ~/workspace_name_ws/src
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

## Tutorials
- There is a tutorial notebook on how to access each recording in the dataset. [CRCD Tutorial](notebooks/CRCD Tutorial)

## Citation

If you found this dataset helpful, please consider citing: 
```
@misc{oh2023comprehensive,
      title={Comprehensive Robotic Cholecystectomy Dataset (CRCD): Integrating Kinematics, Pedal Signals, and Endoscopic Videos}, 
      author={Ki-Hwan Oh and Leonardo Borgioli and Alberto Mangano and Valentina Valle and Marco Di Pangrazio and Francesco Toti and Gioia Pozza and Luciano Ambrosini and Alvaro Ducas and Milos Zefran and Liaohai Chen and Pier Cristoforo Giulianotti},
      year={2023},
      eprint={2312.01183},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
