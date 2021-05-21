%% FANUC
q = zeros(6,1);

dh_params = [0.45, q(1), 0.15, pi/2;
                0, q(2), 0.6, 0;
                0, q(3), 0.2, pi/2;
                0.64, q(4), 0, -pi/2;
                0, q(5), 0, pi/2;
                0, q(6), 0, 0]; 
T01 = trvec2tform([0, 0, dh_params(1,1)]) * rotm2tform(rotz(rad2deg(dh_params(1,2)))) * trvec2tform([dh_params(1,3), 0, 0]) * rotm2tform(rotx(rad2deg(dh_params(1,4)))) 
T12 = trvec2tform([0, 0, dh_params(2,1)]) * rotm2tform(rotz(rad2deg(dh_params(2,2)))) * trvec2tform([dh_params(2,3), 0, 0]) * rotm2tform(rotx(rad2deg(dh_params(2,4))));
T23 = trvec2tform([0, 0, dh_params(3,1)]) * rotm2tform(rotz(rad2deg(dh_params(3,2)))) * trvec2tform([dh_params(3,3), 0, 0]) * rotm2tform(rotx(rad2deg(dh_params(3,4))));
T34 = trvec2tform([0, 0, dh_params(4,1)]) * rotm2tform(rotz(rad2deg(dh_params(4,2)))) * trvec2tform([dh_params(4,3), 0, 0]) * rotm2tform(rotx(rad2deg(dh_params(4,4))));
T45 = trvec2tform([0, 0, dh_params(5,1)]) * rotm2tform(rotz(rad2deg(dh_params(5,2)))) * trvec2tform([dh_params(5,3), 0, 0]) * rotm2tform(rotx(rad2deg(dh_params(5,4))));
T56 = trvec2tform([0, 0, dh_params(6,1)]) * rotm2tform(rotz(rad2deg(dh_params(6,2)))) * trvec2tform([dh_params(6,3), 0, 0]) * rotm2tform(rotx(rad2deg(dh_params(6,4))));

T46 = T45*T56;
T36 = T34*T46;
T26 = T23*T36;
T16 = T12*T26;
T06 = T01*T16;

%% R rotation
R56 = tform2rotm(T56)
R46 = tform2rotm(T46);
R36 = tform2rotm(T36);
R26 = tform2rotm(T26);
R16 = tform2rotm(T16);
R06 = tform2rotm(T06);

%% t traslation
t56 = tform2trvec(T56)
t46 = tform2trvec(T46);
t36 = tform2trvec(T36);
t26 = tform2trvec(T26);
t16 = tform2trvec(T16);
t06 = tform2trvec(T06);

% RPY 
RPY56 = rotm2eul(R56, 'ZYX') % X-Y-X wrt to moving frame
RPY46 = rotm2eul(R46, 'ZYX');
RPY36 = rotm2eul(R36, 'ZYX');
RPY26 = rotm2eul(R26, 'ZYX');
RPY16 = rotm2eul(R16, 'ZYX');
RPY06 = rotm2eul(R06, 'ZYX');

% AXIS - ANGLE 
aa56 = rotm2axang(R56)
aa46 = rotm2axang(R46);
aa36 = rotm2axang(R36);
aa26 = rotm2axang(R26);
aa16 = rotm2axang(R16);
aa06 = rotm2axang(R06);