package org.firstinspires.ftc.teamcode.robots.taubot.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.taubot.simulation.ServoSim;

public class Joint {
    private Servo motor;
    private double HOME_PWM, PWM_PER_DEGREE, DEG_MIN, DEG_MAX;
    private double interimAngle;
    private double targetAngle, oldTargetAngle;
    private double oldTime;
    String name;
    PwmControl.PwmRange axonRange;
    public static double jointSpeed;

    public Joint(HardwareMap hardwareMap, Servo motor, String name, boolean simulated, double HOME, double PER_DEGREE, double MIN, double MAX, double startAngle, double speed)
    {
        this.motor = motor;
        this.name = name;
        HOME_PWM = HOME;
        PWM_PER_DEGREE = PER_DEGREE;
        DEG_MIN = MIN;
        DEG_MAX = MAX;
        //in degrees per ms, parameter in degrees per second
        jointSpeed = speed / 1000;
        interimAngle = startAngle;
        oldTargetAngle = startAngle;
        axonRange = new PwmControl.PwmRange(500, 2500);
        initJoint(simulated, hardwareMap);
    }

    public void initJoint(boolean simulated, HardwareMap hardwareMap){
        if(simulated) {
            motor = new ServoSim();
        }

        else {
            hardwareMap.get(ServoImplEx.class, name);
            ((ServoImplEx) motor).setPwmRange(axonRange);
        }
    }

    public void setTargetAngle(double angle){
        oldTargetAngle = interimAngle;
        targetAngle = angle;
        oldTime = System.nanoTime() / 1e6;
    }

    public void setTargetAngle(double angle, double speed){
        jointSpeed = speed / 1000;
        oldTargetAngle = interimAngle;
        targetAngle = angle;
        oldTime = System.nanoTime() / 1e6;
    }

    public void setSpeed (double speed){
        //parameter in degrees per second, jointSpeed in ms
        jointSpeed = speed / 1000;
    }

    public void update(){
        double newTime = System.nanoTime() / 1e6;
        if(targetAngle > interimAngle)
            interimAngle = oldTargetAngle + (newTime - oldTime) * jointSpeed;
        else
            interimAngle = oldTargetAngle - (newTime - oldTime) * jointSpeed;
        motor.setPosition(calcTargetPosition(interimAngle));
    }

    private double calcTargetPosition(double targetPos) {
        double newPos = Range.clip(targetPos, DEG_MIN, DEG_MAX);
        newPos = newPos * PWM_PER_DEGREE + HOME_PWM;
        return newPos;
    }

}
