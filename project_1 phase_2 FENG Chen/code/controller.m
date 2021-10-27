function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

e_p = s_des(1:3) - s(1:3);
e_v = s_des(4:6) - s(4:6);

global 	T_pre e_p_int count P_MSE_list V_MSE_list  P_RMS V_RMS des cur des_v cur_v
P_RMS = P_RMS + e_p'*e_p;
V_RMS = V_RMS + e_v'*e_v;
P_MSE_list = [P_MSE_list;e_p'*e_p];
V_MSE_list = [V_MSE_list; e_v'*e_v];
des = [des;s_des(1:3)'*s_des(1:3)];
des_v = [des_v;s_des(4:6)'*s_des(4:6)];
cur = [cur;-s(1:3)'*s(1:3)];
cur_v = [cur_v;-s(4:6)'*s(4:6)];
count = count +  1;

% s(1:3) current position
% s(4:6) current velocity
% s(7:10) current attitude quaternion
% s(11:13) current body angular velocity

% s_des(1:3) desire position
% s_des(4:6) desire velocity
% s_des(7:9) desire acceleration
% s_des(10) desire yaw
% s_des(11) desire yaw rate
R = QuatToRot(s(7:10));
[pitch, roll, yaw] = RotToRPY_ZXY(R);
% hyper-parameters of position loop
K_d1 = 7;
K_d2 = 7;
K_d3 = 21;
K_p1 = 4.5;
K_p2 = 9;
K_p3 = 18;

p_acc_1 = s_des(7) + K_d1*(s_des(4) - s(4)) + K_p1*(s_des(1) - s(1));% x
p_acc_2 = s_des(8) + K_d2*(s_des(5) - s(5)) + K_p2*(s_des(2) - s(2));% x
p_acc_3 = s_des(9) + K_d3*(s_des(6) - s(6)) + K_p3*(s_des(3) - s(3));% z
%%%%%%% Force
F = m*(g + p_acc_3);

pitch_c = (p_acc_1*sin(yaw) - p_acc_2*cos(yaw))/g;
roll_c = (p_acc_1*cos(yaw) + p_acc_2*sin(yaw))/g;
p_c_v = 0.0;% desire pitch velocity is 0.
r_c_v = 0.0;% desire roll velocity is 0.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% hyper-parameters of attitude loop
K_pp = 600;%for pitch
K_dp = 40;
K_pr = 600;%for roll
K_dr = 40;
K_py = 600;%for yaw
K_dy = 40;

p_diff = pitch_c - pitch;
r_diff = roll_c - roll;
y_diff = s_des(10) - yaw;

p_acc_c = K_pp*atan2(sin(p_diff), cos(p_diff)) + K_dp*(p_c_v - s(11));
r_acc_c = K_pr*atan2(sin(r_diff), cos(r_diff)) + K_dr*(r_c_v - s(12));
y_acc_c = K_py*atan2(sin(y_diff), cos(y_diff)) + K_dy*(s_des(11) - s(13));

des_acc = [p_acc_c, r_acc_c, y_acc_c]';
angular_v = [s(11), s(12), s(13)]';
%%%%%%%Moment
M = I*des_acc + cross(angular_v, (I*angular_v));
 % You should calculate the output F and M

end
