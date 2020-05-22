Lab 3 Matlab and Simulink Files details - 

lab3_1DOF.m - passive, PD, PID, Skyhook for 1 DOF system
lab3_1DOF_grid_search.m - performs grid search to find optimal PD values for 1 DOF

lab3_2DOF.m - passive, Skyhook for 2 DOF 
genResponse.m - generates discrete time simulation in MATLAB instead of simulink for 2 DOF Skyhook control, this helps consitent comparision with passive sytem

lab3_vehicle_skhook.m - passive + Skyhook for vehicle model of bounce and pitch control
lab3_vehicle_skyhook_sim - simulates and compares passive and Skyhook system for vehicle model
 
lab3_vehicle_H_inf_ctrl.m - passive + Hinf for vehicle model of bounce and pitch control
lab3_vehicle_H_inf_sim - simulates compares passive and Hinf control for vehicle model

lab3_vehicle_model_uncert - passive + Skyhook + Hinf for vehicle model along with model uncertainity and performance comparision
lab3_vehicle_complete_sim - simulates passive, skyhook and Hinf control for vehicle model
getUpdatedModel - returns updated state space matrices with change in mass, inertia, stiffness