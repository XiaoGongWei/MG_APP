function [B,L,H]=XYZ_BLH(X,Y,Z,Cs)

R=sqrt(X*X+Y*Y);
B0=atan2(Z,R);

while 1
    N=Cs.A/sqrt(1-Cs.E2*sin(B0)*sin(B0));
    B=atan2(Z+N*Cs.E2*sin(B0),R);
    if abs(B-B0)<1e-12
        break;
    end;
    B0=B;
    L=atan2(Y,X);
    H=R/cos(B)-N;
end;