param.mass      = 265;
param.h_CG      = 0.300; % height of center of gravity
param.wheelbase = 1.600;
param.track_f = 1.20; % front track
param.h_RC_f  = 0.06;  % height of front roll center
param.track_r = 1.18;  % rear track
param.h_RC_r  = 0.08;  % height of rear roll center
param.LLTD_f = 0.55; % lateral Load Transfer Distribution front


% Load transfer calculation function
% g_mag: magnitude of acceleration in g's.
% acc_dir: acceleration direction in degrees. (0 is front, 90 is left)
function [latf_elastic, latf_geo, latr_elastic, latr_geo, long] = loadTransfer(param, g_mag, acc_dir)

g = 9.81;

m  = param.mass;
hC = param.h_CG;
L  = param.wheelbase;
tf = param.track_f;
hRf = param.h_RC_f;
tr = param.track_r;
hRr = param.h_RC_r;
LLTD_f = param.LLTD_f;

a  = g_mag * g;
ax = a * cosd(acc_dir);
ay = a * sind(acc_dir);

latf_geo = -m * ay * hRf / tf;
latr_geo = -m * ay * hRr / tr;

hR_avg = (hRf + hRr) / 2;
latf_elastic = -m * ay * (hC - hR_avg) * LLTD_f/ tf;
latr_elastic = -m * ay* (hC - hR_avg) *(1-LLTD_f)/ tr;

long = -m * ax * hC / L;

end

g_mag   = input("g magnitude:");
acc_dir = input("acceleration direction (0-360, 0 is front, 90 is left):");

[latf_elastic, latf_geo, latr_elastic, latr_geo, long] = loadTransfer(param,g_mag,acc_dir);

latf_total = latf_geo + latf_elastic;
latr_total = latr_geo + latr_elastic;
lat_total = latf_total + latr_total;

disp("lateral front geometric (N):" + latf_geo)
disp("lateral front elastic (N):" + latf_elastic)
disp("lateral rear geometric (N):" + latr_geo)
disp("lateral rear elastic (N):" + latr_elastic)
disp("lateral front (N):" + latf_total)
disp("lateral rear (N):" + latr_total)
disp("lateral total (N):" + lat_total)
disp("longitudinal (N):" + long)
disp("negative means load transfer to right or to the back.")


