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
    String name;
    PwmControl.PwmRange axonRange;
    public static double JOINT_SPEED;

    public Joint(HardwareMap hardwareMap, Servo motor, String name, boolean simulated, double HOME, double PER_DEGREE, double MIN, double MAX, double startAngle)
    {
        this.motor = motor;
        this.name = name;
        HOME_PWM = HOME;
        PWM_PER_DEGREE = PER_DEGREE;
        DEG_MIN = MIN;
        DEG_MAX = MAX;
        JOINT_SPEED = 10;
        interimAngle = startAngle;
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
        targetAngle = angle;

    }

    public void update(){

    }

    private double calcTargetPosition(double targetPos) {
        double newPos = Range.clip(targetPos, DEG_MIN, DEG_MAX);
        newPos = newPos * PWM_PER_DEGREE + HOME_PWM;
        return newPos;
    }

}
