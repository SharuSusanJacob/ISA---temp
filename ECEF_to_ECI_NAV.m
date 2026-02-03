function [r_ECI, v_ECI] = ECEF_to_ECI_NAV(r_ECEFin, v_ECEFin,GNSSWeek,TOW,leap_seconds)

% Step 1: UTC time
gps_epoch = datetime(1980,1,6,0,0,0);
gps_seconds = GNSSWeek * 7 * 86400 + TOW;
UTC = gps_epoch + seconds(gps_seconds - leap_seconds);
longitude = 0;

[yr, mo, dy, hr, mn, sc] = datevec(UTC);

%IST = UTC + seconds(19800);
% Step 2: Julian Date
year   = yr;
month  = mo;
day    = dy;
hour   = hr;
minute = mn;
second = sc;


if month <= 2
    year = year - 1;
    month = month + 12;
end

A = floor(year / 100);
B = 2 - A + floor(A / 4);
JD = floor(365.25 * (year + 4716)) + ...
     floor(30.6001 * (month + 1)) + ...
     day + B - 1524.5 + (hour + minute/60 + second/3600)/24;
t = (JD - 2451545.0) / 36525;
GMST = 280.46061837 + 360.98564736629 * (JD - 2451545) + t^2 * (0.000387933 - t) / 38710000;
GMST = mod(GMST, 360);  % Normalize degrees
LST_deg = mod(GMST + longitude, 360);
LST_rad = deg2rad(LST_deg);
R_ecef2eci = [cos(LST_rad), -sin(LST_rad), 0;
              sin(LST_rad),  cos(LST_rad), 0;
              0,             0,            1];

omega_earth = 7.2921150e-5;  % [rad/s]
omega_vec = [0; 0; omega_earth];


r_ECI = R_ecef2eci * r_ECEFin;
v_ECI = R_ecef2eci * v_ECEFin + cross(omega_vec, r_ECI);


end
