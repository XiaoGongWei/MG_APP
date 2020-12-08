function Cs=GetCoordSys(type)

Cs.type='1984';%椭球名称，可以为：‘1954’，‘1980’，‘1984’等
Cs.A=6378137.0;%椭球长半径；
Cs.Alfa=1.0/298.257223563;%椭球扁率；
Cs.E2=0.00669437999014132;%椭球第一偏心率的平方；
Cs.UTM=1.0;%中央子午线的经度(用弧度表示)；
Cs.X0=0;Cs.Y0=0;%X0,Y0分别为X、Y坐标加常数(以km为单位)
Cs.L0=0;%为中央子午线经度(用弧度表示)；
Cs.H0=0;%为投影面高程(此处为大地高，以m为单位)
Cs.DN=0;%高程异常(以m为单位);
Cs.GM=0;
Cs.Omega=0;
switch type
    case '1984'
        return
    case '1969'
        Cs.type='1969';
        Cs.A=6378160.0;
        Cs.Alfa=1.0/298.25;
        Cs.E2=2*Cs.Alfa*Cs.Alfa;
        Cs.GM=0;
        Cs.Omega=0;
        Cs.UTM=0.9996;
    case '1980'
        Cs.type='1980';
        Cs.A=6378140.0;
        Cs.UTM=1.0;
        Cs.Alfa=1.0/298.257;
        Cs.GM=0;
        Cs.Omega=0;
        Cs.E2=0.006694384999588;
    case '1954'
        Cs.type='1954';
        Cs.A=6378245.0;
        Cs.Alfa=1.0/298.3;
        Cs.E2=0.00669342162296594;
        Cs.GM=0;
        Cs.Omega=0;
        Cs.UTM=1.0;
    case '2000'
        Cs.type='2000';
        Cs.A=6378137.0;
        Cs.Alfa=1.0/298.257222101;
        Cs.E2=2*Cs.Alfa*Cs.Alfa;
        Cs.GM=3.986004418*1e+14;
        Cs.Omega=7.292115*1e-5;
        Cs.UTM=0;
    case 'PZ-90'
        Cs.type='PZ-90';
        Cs.A=6378136.0;
        Cs.Alfa=1.0/298.257839303;
        Cs.E2=2*Cs.Alfa*Cs.Alfa;
        Cs.GM=3.9860044*1e+14;
        Cs.Omega=7.292115*1e-5;
        Cs.UTM=0;
    otherwise
        disp('没有此椭球模型的参数，已经将所有参数设置为WGS－84的参数！');
end;

