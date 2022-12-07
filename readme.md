# Brief introduction

Codes for the paper with the title of "General Driving Behavior Model based on the Desired Safety Margin for Vehicle Flow Simulation" (ITSC 2022).

This paper establishes a general driving behavior model that is applied in car-following, lane-changing and merging scenarios.

Referring to the NGSIM dataset, this paper realizes the similar-to-real vehicle flow simulation by the self-developed simulation platform based on PyGame. 

# Simulation video

[https://www.bilibili.com/video/BV1Am4y1d7WT/](https://www.bilibili.com/video/BV1Am4y1d7WT/)
![image](https://user-images.githubusercontent.com/24663258/206148572-9e3c760e-1b28-4a40-b7e2-e6c493996514.png)


# Installation requirements

pygame 2.1.2 (SDL 2.0.16, Python 3.9.12)


# Tree view of code
```
GDBDSM-main/
|—— model
|   |—— GDM                       - General Driving Behavior Model
|   |   |—— simulation.py         - main program
|   |   |—— roadInfo.py           - presupposed information
|   |   |—— carflow.py            - generate flow randomly 
|   |   |—— get_flow.py           - get flow from dataset
|   |   |—— car_behavior.py       - car module
|   |   |—— SRP.py                - subjective risk perception
|   |   |—— dubins.py             - dubins curves
|   |   |—— pkl_period_x          - record data of simulation (x=1,2,3)
|   |   |—— visual.py             - replay the simulation from recording data
|   |   |—— velocity_sum          - record veloctiy of simulation
|   |   |—— exam_for_mfd.py       - simulations under different demands
|   |   |—— result                - result of exam_for_mfd.py 
|   |—— IDM+MOBIL                 - baseline model
|   |   |—— IDM_MOBIL.py          - main program
|   |   |—— roadInfo.py           - presupposed information
|   |   |—— velocity_sum          - record veloctiy of simulation
|—— I80_data_process              - initial states of vehicles in dataset (lane id, time, speed)
|   |—— init_vehicle_1.csv        - period 1
|   |—— init_vehicle_2.csv        - period 2
|   |—— init_vehicle_3.csv        - period 3
|—— analyze                       - plot figures in our paper
|—— readme.md
```


# Citation

If you use the codes, please consider citing the following publication:
```
@inproceedings{cai2022general,
  title={General Driving Behavior Model based on the Desired Safety Margin for Vehicle Flow Simulation},
  author={Cai, Pinlong and Zhang, Junjie and Zhao, Xuan and Li, Yikang},
  booktitle={2022 IEEE 25th International Conference on Intelligent Transportation Systems (ITSC)},
  pages={743--748},
  year={2022},
  organization={IEEE}
}
```
# Acknowledgement

The data from Next Generation SIMulation (NGSIM) program is used for this study.

# Contact

If you require any further information, please feel free to contact me by E-mail (caipinlong@pjlab.org.cn). 

