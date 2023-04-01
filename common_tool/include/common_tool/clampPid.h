#pragma once
#include<limits>
class clampPid
{
public:
    double Kp;
    double Ki;
    double Kd;    
    double deltaTime;
    double downLimit;
    double upLimit;
    double IntergrationDownLimit;
    double IntergrationUplimit;
    double Intergration;
    double output;
    clampPid(double Kp_local=0,double Ki_local=0,double Kd_local=0,double deltaTime_local=0);
    clampPid( double Kp_local,double Ki_local,double Kd_local,\
              double deltaTime_local,\
              double downLimit_local,\
              double upLimit_local,\
              double IntergrationDownLimit_local,\
              double  IntergrationUplimit_local );
    void resetIntergration();  
    double step(double error);  
    void setKp(double Kp_local);
    void setKi(double Ki_local);
    void setDeltaTime(double deltaTime_local); 
    void setAllParameter(double Kp_local,double Ki_local,double Kd_local,\
              double deltaTime_local,\
              double downLimit_local,\
              double upLimit_local,\
              double IntergrationDownLimit_local,\
              double  IntergrationUplimit_local);
private:    
};
