# Combine-KalmanNet-and-Value-Iteration-Algorithm
In this scheme, we try to combine three process of design a control system scheme, **system identification-state estimation-optimal control design**. First of all, the system identification process was done using Recurrent Neural Network scheme from the **plant's measured data**. RNN could represent the SS model of a dynamic system, in this reporsitory we try two study case. First study case was ***batch distillation process*** and the second one was ***speed control of DC Motor***. 

![scheme](https://user-images.githubusercontent.com/46149713/179192901-36828d9a-a975-413c-a186-256122e8a1b7.jpg)
# Tutorial 
1. Open file 'main.m' in MATLAB 
2. Choose the system you would try
3. The system's model was defined from system identification process using Recurrent Neural Network scheme. If you would like to do it by your owm, you could use the dataset from this reporsitory (data_distillate.csv for 1st study case and data_motor.csv for 2nd study case)
