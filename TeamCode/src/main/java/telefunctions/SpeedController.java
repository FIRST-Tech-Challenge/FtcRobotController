package telefunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import util.PID;

public class SpeedController {
    ElapsedTime timer = new ElapsedTime();
    PID pid = new PID();

    double lastPos;
    double lastTime;
    public double lastSpeed;

    public double currError = 0;
    public double currAccel = 0;
    public double integralOfError = 0;

    public double targetSpeed = 0;

    double changeTime = 0;

    double oldPos = 0;

    public double pow = 0;

    public SpeedController(double k, double d, double i){
        k *= 0.00001;
        d *= 0.00001;
        i *=  0.00001;
        timer.reset();
        lastPos = 0;
        lastTime = -0.1;
        lastSpeed = 0;
        oldPos = 0;
        pid.setCoeffecients(k,d,i);
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

    public void updateMotorValues(double currPos){
        double currSpeed = getMotorSpeed(currPos);
        currError = targetSpeed-currSpeed;
        currAccel = currError/changeTime;
        lastSpeed = currSpeed;
        integralOfError += currError*changeTime;
    }

    public double getPercentageError(){
        return ((currError/targetSpeed))*100;
    }

    public double getPow(){
        pow += Math.signum(currError) * pid.getPower(currError, currAccel, integralOfError);
        pow = Range.clip(pow, -1, 1);
        return pow;
    }


}
