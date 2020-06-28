# The version description of MG-APP 

## MG_APP_V1.0.0.exe

- Please use the [https://github.com/XiaoGongWei/MG_APP](https://github.com/XiaoGongWei/MG_APP) link code to get the latest features.
- PPP of Ion-Free combination.
- Use Kalman and SRIF filtering.
- Support forward and backward filtering.
- Support GPS/BDS/GLONASS/Galileo system data solution.
## MG_APP_V2.0.0.exe

- Support all functions of V1.0.0.
- Support for Uncombination PPP.
- Support for final products (GBM(first) or IGS) and CNT products (save real-time data ephemeris). 
- Support real-time display of positioning in Baidu map.

## MG_APP_V2.0.1.exe

- Support all functions of V2.0.0.
- Update antenna igs14_2101.atx file for BDS3.
- Support BDS3 (B2I and B6I) data processing. The processing results:    [https://blog.csdn.net/xiaoxiao133/article/details/105561824](https://blog.csdn.net/xiaoxiao133/article/details/105561824)
- Fixed some bugs of backward filtering.

## MG_APP_V2.0.2.exe

- Support all functions of V2.0.1.
- ~~Support software update automatically (MG-APP -> Tools -> Update, only in Windows system).~~
  ~~If there is no update software, MG-APP will restart once, if there is an update, it will download the  update file.~~

## MG_APP_V2.0.3.exe

- Support all functions of V2.0.1.
- Support software update automatically (MG-APP -> Tools -> Update, in Windows/Linux etc. )
  If there is no update software, MG-APP will restart once, if there is an update, it will download the  update file and restart.
  The software update function is developed by the Gongwei Xiao using Qt.
- Fix read observation data(QReadOFile.cpp).

## MG_APP_V2.0.4.exe

- Support all functions of V2.0.3.
- Added the function of deleting satellite and selecting the observed value type.
- Added configuration files (.ini and .json) for easy network transport.
- Please use the Update function (<u>MG-APP -> Tools -> Update</u>) of version <u>2.0.3</u> to upgrade to 2.0.4.

## MG_APP_V2.0.5.exe

- Support all functions of V2.0.4.
- Added GUI for setting Transfer of noise (Qw) and Initial covariance (Pk) in configuration files (.ini and .json) .
- Please use the Update function (<u>MG-APP -> Tools -> Update</u>) of version <u>2.0.3</u>  or  <u>2.0.4</u> to upgrade to 2.0.5.
- The MG-APP V2.0.5 source code can be **<u>open source</u>** for **MG-APP developers**, but developers must be **proficient** in using github's [MG-APP](https://github.com/XiaoGongWei/MG_APP) source code and sign a source code **confidentiality agreement**.

Author: Gongwei Xiao

 QQ Group: **258113285**

 E-mail: [xiaogongwei10@163.com](xiaogongwei10@163.com)

 My GitHub: https://github.com/xiaogongwei

 My Blog: [xiaogongwei10.blog.csdn.net](xiaogongwei10.blog.csdn.net)