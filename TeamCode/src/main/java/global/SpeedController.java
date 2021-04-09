package global;

import com.qualcomm.robotcore.util.ElapsedTime;

import globalfunctions.Constants;
import globalfunctions.PID;

public class SpeedController {

    ElapsedTime timer = new ElapsedTime();

    PID pid = new PID();

    double lastPos;
    double lastTime;



    public double derivativeOfPower = 0;
    public double power = 0;

    public double targetSpeed = 0; // rad/s
    public double currSpeed = 0;

    double changeTime = 0;


    public boolean isReady = false;


    public SpeedController(){
        lastTime = -0.1;
        pid.Kp = 0.008;
        pid.Kd = 0.002;
        pid.Ki = 0.000;
//        pid.Ki = 0.0001;
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
        lastTime = -0.1;
        lastPos = currPos;
        power = targetSpeed/Constants.MAX_OUTTAKE_SPEED;
        timer.reset();
    }

    public double getMotorPower(double currPos){
        updateMotorSpeed(currPos);

        pid.update(targetSpeed-currSpeed);

        derivativeOfPower = pid.getPower();
        power += derivativeOfPower*changeTime;
        return power;
    }

    public boolean isReady(){
        return isReady;
    }


}
