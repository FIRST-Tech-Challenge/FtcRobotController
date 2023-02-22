package org.firstinspires.ftc.teamcode.robots.taubot.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
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
    public static double jointSpeed;

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
        //in degrees per ms, parameter in degrees per second
        jointSpeed = speed / 1000;
        interimAngle = startAngle;
        oldTargetAngle = startAngle;
        initJoint(simulated, hardwareMap);
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

    public void setTargetAngle(double angle){
        //new starting position is current servo pos
        oldTargetAngle = interimAngle;
        targetAngle = angle;
        if(interimAngle > targetAngle)
            targetIsHigher = false;
        else
            targetIsHigher = true;
        //starts timer to reach target angle
        oldTime = System.nanoTime() / 1e6;
    }

    public void setTargetAngle(double angle, double speed){
        jointSpeed = speed / 1000;
        oldTargetAngle = interimAngle;
        targetAngle = angle;
        oldTime = System.nanoTime() / 1e6;
    }

    public double getPosition(){
        return motor.getPosition();
    }

    public void setSpeed (double speed){
        //parameter in degrees per second, jointSpeed in ms for smoothness
        jointSpeed = speed / 1000;
    }

    public void update(){
        double newTime = System.nanoTime() / 1e6;
        //calculates position based on time passed since angle set and degrees per ms
        if(targetAngle < interimAngle && !targetIsHigher) {
            interimAngle = oldTargetAngle - (newTime - oldTime) * jointSpeed;
        }
        else if (targetAngle > interimAngle && targetIsHigher) {
            interimAngle = oldTargetAngle + (newTime - oldTime) * jointSpeed;
        }
        //once target is reached or exceeded, servo is told to go to targetAngle
        if (targetIsHigher && interimAngle > targetAngle)
            interimAngle = targetAngle;
        if(!targetIsHigher && interimAngle < targetAngle)
            interimAngle = targetAngle;

        motor.setPosition(calcTargetPosition(interimAngle));
    }

    private double calcTargetPosition(double targetPos) {
        //angle to servo unit conversion with min and max clip
        double newPos = Range.clip(targetPos, DEG_MIN, DEG_MAX);
        newPos = newPos * PWM_PER_DEGREE + HOME_PWM;
        return newPos;
    }

}
