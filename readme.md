# Brief introduction

Codes for the paper with the title of "General Driving Behavior Model based on the Desired Safety Margin for Vehicle Flow Simulation" (ITSC 2022).

This paper establishes a general driving behavior model that is applied in car-following, lane-changing and merging scenarios.

Referring to the NGSIM dataset, this paper realizes the similar-to-real vehicle flow simulation by the self-developed simulation platform based on PyGame. 

# Simulation video

[https://www.bilibili.com/video/BV1Am4y1d7WT/](https://www.bilibili.com/video/BV1Am4y1d7WT/)
![image](https://user-images.githubusercontent.com/24663258/206148572-9e3c760e-1b28-4a40-b7e2-e6c493996514.png)


# Installation Requirements

pygame 2.1.2 (SDL 2.0.16, Python 3.9.12)


# Tree view of code
GDBDSM-main/<br>
|—— model<br>
|&nbsp;|—— GDM                       - General Driving Behavior Model<br>
|&nbsp;|&nbsp;|—— simulation.py         - main program<br>
|&nbsp;|&nbsp;|—— roadInfo.py           - presupposed information<br>
|&nbsp;|&nbsp;|—— carflow.py            - generate flow randomly <br>
|&nbsp;|&nbsp;|—— get_flow.py           - get flow from dataset<br>
|&nbsp;|&nbsp;|—— car_behavior.py       - car module
|&nbsp;|&nbsp;|—— SRP.py                - subjective risk perception<br>
|&nbsp;|&nbsp;|—— dubins.py             - dubins curves<br>
|&nbsp;|&nbsp;|—— pkl_period_x          - record data of simulation (x=1,2,3)<br>
|&nbsp;|&nbsp;|—— visual.py             - replay the simulation from recording data<br>
|&nbsp;|&nbsp;|—— velocity_sum          - record veloctiy of simulation<br>
|&nbsp;|&nbsp;|—— exam_for_mfd.py       - simulations under different demands<br>
|&nbsp;|&nbsp;|—— result                - result of exam_for_mfd.py <br>
|&nbsp;|—— IDM+MOBIL                 - baseline model<br>
|&nbsp;|&nbsp;|—— IDM_MOBIL.py          - main program<br>
|&nbsp;|&nbsp;|—— roadInfo.py           - presupposed information<br>
|&nbsp;|&nbsp;|—— velocity_sum          - record veloctiy of simulation<br>
|—— I80_data_process              - initial states of vehicles in dataset (lane id, time, speed)<br>
|&nbsp; |—— init_vehicle_1.csv        - period 1<br>
|&nbsp;|—— init_vehicle_2.csv        - period 2<br>
|&nbsp;|—— init_vehicle_3.csv        - period 3<br>
|—— analyze                       - plot figures in our paper<br>
|—— readme.md


# Citation

If you use the codes, please consider citing the following publication:

@inproceedings{cai2022general,
  title={General Driving Behavior Model based on the Desired Safety Margin for Vehicle Flow Simulation},
  author={Cai, Pinlong and Zhang, Junjie and Zhao, Xuan and Li, Yikang},
  booktitle={2022 IEEE 25th International Conference on Intelligent Transportation Systems (ITSC)},
  pages={743--748},
  year={2022},
  organization={IEEE}
}

# Acknowledgement

The data from Next Generation SIMulation (NGSIM) program is used for this study.

# Contact

If you require any further information, please feel free to contact me by E-mail (caipinlong@pjlab.org.cn). 

