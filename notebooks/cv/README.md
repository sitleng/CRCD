# Tutorial Notebooks

## Intro

Here are some notebooks to guide for each usage:

- [PointCloud Tutorial](cv/crcd_pcl.ipynb): 3D reconstruction example with the stereo endoscopic images.

- [Detectron2 Segmentation](cv/crcd_detectron2_segmentation.ipynb): Shows how to load the annotated tissue segmentation dataset and fine tune the model.

- [Detectron2 Keypoints](cv/crcd_detectron2_keypoints.ipynb): Shows how to load the annotated instrument keypoint dataset and fine tune the model.

## Conda environment

It is recommended to create a virtual python environment before installing the OpenCV and Detectron2

- Follow the [official installation steps](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html) for conda.

- Create a conda environment. Feel free to change the python version.
```
conda create -n crcd python=3.10
```

- Let's activate the environment!
```
conda activate crcd
```

## OpenCV CUDA Build

OpenCV is recommended to be built with CUDA enabled to generate point clouds.

Reference: https://github.com/opencv/opencv-python

You can follow these steps tested on Ubuntu 20.04.

1. Clone repository:
```
git clone --recursive https://github.com/opencv/opencv-python.git
```

2. Go to the OpenCV directory:
```
cd opencv-python
```

3. Add custom Cmake flags. Here is an example for the Nvidia RTX 4060 ti.
   
   (check the cuDNN at https://developer.nvidia.com/cuda-gpus):
   
```
export CMAKE_ARGS="-D CMAKE_BUILD_TYPE=RELEASE \
	-D WITH_CUBLAS=ON \
	-D WITH_CUDA=ON \
	-D WITH_CUDNN=ON \
	-D BUILD_opencv_cudacodec=OFF \
	-D BUILD_opencv_cudaoptflow=OFF \
	-D OPENCV_DNN_CUDA=ON \
	-D PARALLEL_ENABLE_PLUGINS=OFF \
	-D CUDA_ARCH_BIN=8.9 \
	-D WITH_OPENGL=ON \
	-D OPENCV_ENABLE_NONFREE=ON .."
```

4. Select the package flavor which you wish to build with ENABLE_CONTRIB and ENABLE_HEADLESS:
```
export ENABLE_CONTRIB=1
```

5. Make sure you have the latest pip version and run:
```
python3 -m pip install --upgrade pip
pip3 wheel . --verbose
```

6. Check the desired python environment path from the list:
```
echo $PYTHONPATH
```

8. Unzip the wheel to one of the PYTHONPATH locations:

   (e.g., assuming you installed OpenCV version 4.8.1)
```
unzip opencv_contrib_python-4.8.1.78-cp39-cp39-linux_x86_64.whl -d /home/$USER/anaconda3/envs/<your_conda_env>/lib/<conda_env_python_version>/site-packages
```

## Detectron2 Setup

It is required to setup a virtual environment with detectron2 installed to safely run the Detectron2 notebooks.

1. 


Reference: https://detectron2.readthedocs.io/en/latest/tutorials/install.html



You can follow these steps tested on Ubuntu 20.04.


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
