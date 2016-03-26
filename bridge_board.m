close all
clear
clc


% design variable
b = 30;
h = 45;
h_t = h + 4*1.27;
a_dia = 163.5;
a_side = 300;
cw = 55; %column width

% material property
miu = 0.2;
bbbbb = 1-miu^2;
E = 4000;

% testing variable
P = 1047;
P_column = 1.6 * P;
load_total = 2*P;
V = (1.6/2)*P;
V_side = 0.2*P;


% I 

span_width = 95;
cm1 = 1.27;
cm2 = 1.27/2 + 1.27*2;
cm3 = h/2 + 3*1.27;
cm4 = 1.27/2 + h + 3*1.27;
cm5 = 15/2 + (h-15) + 1.27*3;
cm6 = 1.27/2 + (h-15-1.27)+ 1.27*3;

a1 = 1.27*2*95;
a2 = 1.27*2*b;
a3 = 1.27*4*h;
a4 = 1.27*95;
a5 = 15*1.27*2;
a6 = (95-b)*1.27;

a = a1+a2+a3+a4+a5+a6;
cm = (cm1*a1 + cm2*a2 + cm3*a3 + cm4*a4 + cm5*a5 + cm6*a6)/a;

i1 = a1*(cm1-cm)^2 + 95*(2.54^3)/12;
i2 = a2*(cm2-cm)^2 + 2*b*(1.75^3)/12;
i3 = a3*(cm3-cm)^2 + 1.75*4*(h^3)/12;
i4 = a4*(cm4-cm)^2 + 95*(1.75^3)/12;
i5 = a5*(cm5-cm)^2 + (1.27*2)*(15^3)/12;
i6 = a6*(cm6-cm)^2 + (95-b)*(1.75^3)/12;

I = i1+i2+i3+i4+i5+i6;

% compression stress

M = 60*P; % ----
y = 1.27*4 + h - cm;
Stress_c = M*y/I;

Mb = 96*P;
yb = cm;
Stress_bottom = Mb*yb/I;

% glue stress between component 1 and 2

b_glue_12 = 2*b;
q_glue_12 = 2.54* 95 *(cm-cm1);
tau_glue_12 = V*q_glue_12/I/b_glue_12;

% glue stress between two main decks

b_glue_1 = 95;
q_glue_1 = 1.27 * b_glue_1 * (cm-1.27/2);
tau_glue_1 = V * q_glue_1/I/b_glue_1;

%
h_stress = h_t-cm;
h_longi = h+1.27*3 - cm;
%

% longitudal sheear stress ??, actual shear stress, ???tau_cr
q_longi = 95*1.27*(1.27/2 + h_longi) + 4*1.27*(h_longi)*((h_longi)/2) + 15*2*1.27*(7.5 + (h_longi-15)) + (95-2*b)*1.27*(1.27/2 + (h_longi-15-1.27));
Tau_longi = V*q_longi/I/(1.27*4);
Tau_longi_side = V_side*q_longi/I/(1.27*4);


% stress top flange (compression stress)

Stress_tf1 = 4*(pi^2)*E*((1.27/(95-2*b))^2)/12/bbbbb;
Stress_tf2 = 4*(pi^2)*E*((1.27/(b-2*1.27))^2)/12/bbbbb;

% stress web flexual (compression stress)

Stress_wf = (6/12)*(pi^2)*E*((1.27/h_stress)^2)/bbbbb;

% shear stress recalls shear buckling

Tau_cr = (5/12)*(pi^2)*E*((1.27/h)^2+(1.27/a_dia)^2)/bbbbb;
Tau_cr_side = (5/12)*(pi^2)*E*((1.27/h)^2+(1.27/a_side)^2)/bbbbb;

% compression stress, column
Stress_column_c = 1.6*P/(4*1.27*cw)

% deflection


P_d = 250;
curv = P_d*248/(E*I);
deflection = 0.5*248*curv*(2/3)*248;

% check
P_column_max = pi^2 * 4000 / 600^2 * (cw^4 - (cw-2.54)^4) / 12;
Board_area = 1016*812.8;
B_Width_used = 95*2+(b+h)*4+(15+95-2*b)*2 + 4*cw
B_for_diaphram = 812.8-(95*2+(b+h)*4+(15+95-2*b)*2)
Board_used = 1016*(95*2+(b+h)*4+(15+95-2*b)*2) + b*h*12 + 4*cw*600;

Check1_c_stress = Stress_c < 6
Check2_glue_stress1 = tau_glue_12 < 2
Check3_glue_stress2 = tau_glue_1 < 2
Check4_max_longi_shear_stress = Tau_longi < 4
check5_max_longi_shear_stress_side = Tau_longi_side < 4
Check6_column = P_column_max > P_column
Check7_1_top_flange = Stress_tf1 > Stress_c
Check7_2_top_flange = Stress_tf2 > Stress_c
Check8_web_flex = Stress_wf > Stress_c
Check9_web_sheer = Tau_cr > Tau_longi
Check10_column_stress = Stress_column_c < 6
Check11_board = B_Width_used < 812
Check12_deflection = deflection < 1062/2/50
% Check5_material = Board > Board_used
% max load carried, strength-to-weight ratio, stiffness-to-weight ratio
