package developing;

import com.qualcomm.robotcore.util.ElapsedTime;

import global.Constants;
import util.PID;

public class SpeedController2 {

    ElapsedTime timer = new ElapsedTime();

    PID pid = new PID();

    double lastPos;
    double lastTime;
    public double lastError;

    public double currError = 0;
    public double derivativeOfError = 0;
    public double integralOfError = 0;

    public double derivativeOfPower = 0;
    public double power = 0;

    public double targetSpeed = 0; // rad/s
    public double currSpeed = 0;

    double changeTime = 0;


    public double k;
    public double d;
    public double i;

    public boolean isReady = false;


    public SpeedController2(){
        lastError = -0.1;
        lastTime = -0.1;
        pid.Kp = 0.003;
        pid.Kd = 0.0001;
        pid.Ki = 0.0001;
    }

    public void updateMotorSpeed(double currPos){
        double changePos = currPos-lastPos;
        lastPos = currPos;
        changeTime = timer.seconds()-lastTime;
        lastTime = timer.seconds();
        currSpeed = changePos/changeTime;
    }

    public void setTargetSpeed(double ts){
        targetSpeed = ts;
    }

    public void reset(double currPos){
        lastError = -0.1;
        lastTime = -0.1;
        integralOfError = 0;
        lastPos = currPos;
        power = targetSpeed/Constants.MAX_OUTTAKE_SPEED;
        timer.reset();
    }

    public double getMotorPower(double currPos){
        updateMotorSpeed(currPos);

        currError = targetSpeed-currSpeed;

        derivativeOfError = (currError-lastError)/changeTime;
        lastError = currError;
        integralOfError += currError*changeTime;

        derivativeOfPower = pid.getPower(currError, derivativeOfError, integralOfError);
        power += derivativeOfPower*changeTime;
        return power;
    }

    public boolean isReady(){
        return isReady;
    }


}
