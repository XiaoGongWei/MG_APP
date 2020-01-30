function [NEU,T]=XYZ_NEU(obspos,XYZ)

Cs=GetCoordSys('1984');
[B,L,H]=XYZ_BLH(obspos(1),obspos(2),obspos(3),Cs);
T=[-sin(B)*cos(L) -sin(B)*sin(L) cos(B);-sin(L) cos(L) 0;cos(B)*cos(L) cos(B)*sin(L) sin(B)];
NEU=[];
if size(XYZ,1)==3
    if size(XYZ,2)==1
        NEU=[-sin(B)*cos(L) -sin(B)*sin(L) cos(B);-sin(L) cos(L) 0;cos(B)*cos(L) cos(B)*sin(L) sin(B)]*XYZ;
    else
        for i=1:size(XYZ,2)
            neu=[-sin(B)*cos(L) -sin(B)*sin(L) cos(B);-sin(L) cos(L) 0;cos(B)*cos(L) cos(B)*sin(L) sin(B)]*[XYZ(1,i);XYZ(2,i);XYZ(3,i)];
            NEU=[NEU;neu'];
        end;
    end;
else
    if size(XYZ,1)==1
        NEU=[-sin(B)*cos(L) -sin(B)*sin(L) cos(B);-sin(L) cos(L) 0;cos(B)*cos(L) cos(B)*sin(L) sin(B)]*XYZ';
    else
         for i=1:size(XYZ,2)
            neu=[-sin(B)*cos(L) -sin(B)*sin(L) cos(B);-sin(L) cos(L) 0;cos(B)*cos(L) cos(B)*sin(L) sin(B)]*[XYZ(i,1);XYZ(i,2);XYZ(i,3)];
            NEU=[NEU;neu'];
        end;
    end;
end;