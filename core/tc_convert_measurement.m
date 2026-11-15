function [z, Rz] = tc_convert_measurement(r, th, sig_r, sig_th)
% TC_CONVERT_MEASUREMENT  Unbiased converted measurement (Lerro & Bar-Shalom 1993; Mo et al. 1998).
%   z  = r [cos th; sin th] / lam,   lam = exp(-sig_th^2/2)   (removes the cos/sin bias)
%   Rz = exact covariance of z given the polar noise (sig_r, sig_th).
% Keeps the estimator LINEAR; at this geometry the nonlinearity index
% r*sig_th^2/sig_r is ~2e-4, so an EKF/UKF would be numerically identical.
  lam  = exp(-sig_th^2/2);
  lam2 = exp(-2*sig_th^2);
  c = cos(th);  s = sin(th);
  k = (lam^-2 - 2)*r^2;
  h = 0.5*(r^2 + sig_r^2);
  z  = r*[c; s]/lam;
  Rz = [k*c^2 + h*(1 + lam2*cos(2*th)),  k*c*s + h*lam2*sin(2*th);
        k*c*s + h*lam2*sin(2*th),        k*s^2 + h*(1 - lam2*cos(2*th))];
  Rz = 0.5*(Rz + Rz');
end
