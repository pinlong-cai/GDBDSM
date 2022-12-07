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
|&emsp;&emsp;|—— GDM&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - General Driving Behavior Model<br>
|&emsp;&emsp;|&emsp;&emsp;|—— simulation.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - main program<br>
|&emsp;&emsp;|&emsp;&emsp;|—— roadInfo.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - presupposed information<br>
|&emsp;&emsp;|&emsp;&emsp;|—— carflow.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- generate flow randomly <br>
|&emsp;&emsp;|&emsp;&emsp;|—— get_flow.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - get flow from dataset<br>
|&emsp;&emsp;|&emsp;&emsp;|—— car_behavior.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - car module<br>
|&emsp;&emsp;|&emsp;&emsp;|—— SRP.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- subjective risk perception<br>
|&emsp;&emsp;|&emsp;&emsp;|—— dubins.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - dubins curves<br>
|&emsp;&emsp;|&emsp;&emsp;|—— pkl_period_x&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- record data of simulation (x=1,2,3)<br>
|&emsp;&emsp;|&emsp;&emsp;|—— visual.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - replay the simulation from recording data<br>
|&emsp;&emsp;|&emsp;&emsp;|—— velocity_sum&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- record veloctiy of simulation<br>
|&emsp;&emsp;|&emsp;&emsp;|—— exam_for_mfd.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - simulations under different demands<br>
|&emsp;&emsp;|&emsp;&emsp;|—— result&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- result of exam_for_mfd.py <br>
|&emsp;&emsp;|—— IDM+MOBIL&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - baseline model<br>
|&emsp;&emsp;|&emsp;&emsp;|—— IDM_MOBIL.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- main program<br>
|&emsp;&emsp;|&emsp;&emsp;|—— roadInfo.py&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - presupposed information<br>
|&emsp;&emsp;|&emsp;&emsp;|—— velocity_sum&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- record veloctiy of simulation<br>
|—— I80_data_process&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- initial states of vehicles in dataset (lane id, time, speed)<br>
|&emsp;&emsp;|—— init_vehicle_1.csv&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- period 1<br>
|&emsp;&emsp;|—— init_vehicle_2.csv&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- period 2<br>
|&emsp;&emsp;|—— init_vehicle_3.csv&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;- period 3<br>
|—— analyze&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; - plot figures in our paper<br>
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

