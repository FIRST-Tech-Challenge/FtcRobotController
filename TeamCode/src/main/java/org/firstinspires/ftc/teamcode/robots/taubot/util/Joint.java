package org.firstinspires.ftc.teamcode.robots.taubot.util;

import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.servoNormalize;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.taubot.simulation.ServoSim;

public class Joint {

    private Servo motor = null;
    private double HOME_PWM, PWM_PER_DEGREE, DEG_MIN, DEG_MAX;
    private double interimAngle;
    private double targetAngle, oldTargetAngle;
    private double oldTime;
    private boolean targetIsHigher;
    String name;
    private double jointSpeed;

    //todo: possible issue - servo doesn't reach interimAngle and new setAngle assumes it has
    //todo: add servo feedback functionality
    //todo if necessary: cap min and max speed for another safety layer after testing

    public Joint(HardwareMap hardwareMap, String name, boolean simulated, double HOME, double PER_DEGREE, double MIN, double MAX, double startAngle, double speed)
    {
        this.name = name;
        HOME_PWM = HOME;
        PWM_PER_DEGREE = PER_DEGREE;
        DEG_MIN = MIN;
        DEG_MAX = MAX;
        //parameter in degrees per second
        jointSpeed = speed;
        interimAngle = startAngle;
        oldTargetAngle = startAngle;
        targetAngle = startAngle;
        initJoint(simulated, hardwareMap);
    }

    public void setPWM_PER_DEGREE(double PWM){
        PWM_PER_DEGREE = PWM;
    }

    public void initJoint(boolean simulated, HardwareMap hardwareMap){
        if(simulated) {
            motor = new ServoSim();
        }

        else {
            motor = hardwareMap.get(ServoImplEx.class, name);
            ((ServoImplEx) motor).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    public void RelaxJoint(){((ServoImplEx) motor).setPwmDisable();}
    public void ActivateJoint(){((ServoImplEx) motor).setPwmEnable();}
    public boolean IsActive(){return ((ServoImplEx) motor).isPwmEnabled();}

    public void setTargetAngle(double angle){
        targetAngle = Range.clip(angle, DEG_MIN, DEG_MAX);
    }

    public void setTargetAngle(double angle, double speed){
        jointSpeed = speed;
        setTargetAngle(angle);
    }

    public double getPosition(){
        return motor.getPosition();
    }

    public double getCurrentAngle() {
        //this only works if interimAngle always has the most recently commanded angle actually sent to the servo
        return interimAngle;}

    public double getTargetAngle() { return targetAngle; }

    public void setSpeed (double speed){
        //parameter in degrees per second
        jointSpeed = speed;
    }

    public void update(){

        double newTime= System.nanoTime() / 1e9;
        double deltaTime= newTime-oldTime;
        oldTime = newTime;
        //assume last deltaTime is predictive of next loop, calc interimAngle
        double errAngle = targetAngle-interimAngle;
        if (targetAngle > 30)
            targetAngle = targetAngle;
        interimAngle = Range.clip(interimAngle + Math.signum(errAngle)*jointSpeed * deltaTime,Double.min(targetAngle,interimAngle), Double.max(targetAngle,interimAngle));
        motor.setPosition(servoNormalize(calcTargetPosition(interimAngle)));
    }

    private double calcTargetPosition(double targetPos) {
        //angle to servo unit conversion with min and max clip
        double newPos = Range.clip(targetPos, DEG_MIN, DEG_MAX);
        newPos = newPos * PWM_PER_DEGREE + HOME_PWM;
        return newPos;
    }

}
