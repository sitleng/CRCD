# Comprehensive Robotic Cholecystectomy Dataset (CRCD)

The **Comprehensive Robotic Cholecystectomy Dataset (CRCD)** is a large-scale, publicly available dataset for **robot-assisted surgery (RAS) research**.  
It provides synchronized **endoscopic videos, da Vinci surgical robot kinematics, and pedal usage signals**, making it one of the most comprehensive multimodal resources for studying **robotic cholecystectomy procedures**.  

CRCD is designed to support research in:
- **Controls in RAS**
- **Computer vision for RAS**
- **Surgical workflow analysis and phase recognition**
- **AI and machine learning in healthcare**
- **Skill assessment and surgical training**

By integrating visual, kinematic, and control data, CRCD enables work on **instrument tracking, tissue segmentation, activity recognition, and predictive surgical models.**

---

## Publications

by Ki-Hwan Oh, Leonardo Borgioli, Alberto Mangano, Valentina Valle, Marco Di Pangrazio, Francesco Toti, Gioia Pozza, Luciano Ambrosini, Alvaro Ducas, Miloš Žefran, Liaohai Chen, Pier Cristoforo Giulianotti  

[![arXiv](https://img.shields.io/badge/arXiv-Paper-red?logo=arxiv)](https://arxiv.org/abs/2312.01183)  [![IEEE](https://img.shields.io/badge/IEEE-Paper-blue?logo=ieee)](https://ieeexplore.ieee.org/abstract/document/10585836)  [![BibTeX](https://img.shields.io/badge/BibTeX-Citation-orange?logo=bibtex)](https://uofi.box.com/s/0cxpk70we719hxcqsdn3bx05lw9yfsth)  [![Hugging Face](https://img.shields.io/badge/HuggingFace-Dataset-yellow?logo=huggingface)](https://huggingface.co/datasets/SITL-Eng/CRCD)

### Updates
Expanded version of CRCD published in **Journal of Medical Robotics Research (JMRR):**  

[![arXiv](https://img.shields.io/badge/arXiv-Paper-red?logo=arxiv)](https://arxiv.org/abs/2412.12238#)  [![JMRR](https://img.shields.io/badge/JMRR-World%20Scientific-blue)](https://doi.org/10.1142/S2424905X25500060)

---

## Dataset Contents

- **Raw Dataset**  
  Includes **endoscopic videos**, **da Vinci kinematics**, and **console pedal usage**.  
  [Download Link](https://uofi.box.com/s/p3aocj6yzq4ctwc0s635a2dfyk9zdv5j)

- **Annotated Dataset**  
  Contains annotated frames with **tissue segmentation** and **instrument keypoints**.  
  [Download Link](https://uofi.box.com/s/f9bg69ve6fkwktr3o33ahmp620w8jth6)

- **Additional Information**  
  Provides **stereo endoscopic camera calibrations** and **background info of each surgeon**.  
  [Download Link](https://uofi.box.com/s/w65rui5ylm0i4v4jvlkpacpi4q6jkdpe)

- **Synchronized Dataset**  
  Contains synchronized **endoscopic videos**, **da Vinci kinematics**, and **console pedal usage**.
  [![Hugging Face](https://img.shields.io/badge/HuggingFace-Dataset-yellow?logo=huggingface)](https://huggingface.co/datasets/SITL-Eng/CRCD)

---

## Tutorials

Example usage and tutorials are available in the [notebooks/](notebooks/) folder:  
- Loading and preprocessing CRCD data  
- Running segmentation and instrument tracking  
- Using kinematic and pedal signals for workflow analysis  

---

## 📄 License

The dataset and documentation in this repository are licensed under the [CC-BY-4.0](LICENSE-DS).

The source code used to generate or process the data is licensed under the [MIT License](LICENSE).

---

## Citation

If you use CRCD in your research, please cite:

```bibtex
@INPROCEEDINGS{koh2024crcd,
  author={Oh, Ki-Hwan and Borgioli, Leonardo and Mangano, Alberto and Valle, Valentina and Di Pangrazio, Marco and Toti, Francesco and Pozza, Gioia and Ambrosini, Luciano and Ducas, Alvaro and Žefran, Miloš and Chen, Liaohai and Giulianotti, Pier Cristoforo},
  booktitle={2024 International Symposium on Medical Robotics (ISMR)}, 
  title={Comprehensive Robotic Cholecystectomy Dataset (CRCD): Integrating Kinematics, Pedal Signals, and Endoscopic Videos}, 
  year={2024},
  pages={1-7},
  keywords={Medical robotics;Automation;Robot vision systems;Liver;Kinematics;Predictive models;Cameras},
  doi={10.1109/ISMR63436.2024.10585836}
}

@article{doi:10.1142/S2424905X25500060,
  author = {Oh, Ki-Hwan and Borgioli, Leonardo and Mangano, Alberto and Valle, Valentina and Pangrazio, Marco Di and Toti, Francesco and Pozza, Gioia and Ambrosini, Luciano and Ducas, Alvaro and \v{Z}efran, Milo\v{s} and Chen, Liaohai and Giulianotti, Pier Cristoforo},
  title = {Expanded Comprehensive Robotic Cholecystectomy Dataset},
  journal = {Journal of Medical Robotics Research},
  doi = {10.1142/S2424905X25500060},
  URL = {https://doi.org/10.1142/S2424905X25500060}
}
```
