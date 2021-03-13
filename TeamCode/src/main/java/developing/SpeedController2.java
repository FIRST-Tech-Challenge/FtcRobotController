package developing;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import util.PID;

public class SpeedController2 {

    ElapsedTime timer = new ElapsedTime();

    PID pid = new PID();

    double lastPos;
    double lastTime;
    public double lastError;

    public double currError = 0;
    public double currAccel = 0;
    public double integralOfError = 0;

    public double targetSpeed = 0; // radians/s
    public double currSpeed = 0;

    double changeTime = 0;


    public double k;
    public double d;
    public double i;

    public boolean isReady = false;


    public SpeedController2(){
        lastError = -0.1;
        pid.Kp = 0.01;
        pid.Kd = 0.01;
        pid.Ki = 0.01;
    }

    public double getMotorSpeed(double currPos){
        double changePos = currPos-lastPos;
        lastPos = currPos;
        changeTime = timer.seconds()-lastTime;
        lastTime = timer.seconds();
        return changePos/changeTime;
    }

    public void setTargetSpeed(double ts){
        targetSpeed = ts;
    }
    public void reset(){
        integralOfError = 0;
    }


    public double getMotorPower(double currPos){
        currSpeed = getMotorSpeed(currPos);
        currError = targetSpeed-currSpeed;

        currAccel = (currError-lastError)/changeTime;
        lastError = currError;
        integralOfError += currError*changeTime;

        return  pid.getPower(currError, currAccel, integralOfError);
    }

    public boolean isReady(){
        return isReady;
    }


}
