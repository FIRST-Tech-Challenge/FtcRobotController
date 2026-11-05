package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.broombots.sweep.Classes.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

public class Turret implements Subsystem {
    private HardwareMap hardwareMap;
    private Servo turretServo;
    private DcMotor turretEncoder;
    private PIDController pidController;
    private boolean isDisabled = false;
    private double errorTolerance; //TODO: find a good error tolerance
    private final double ECODER_TICKS_TO_DEGREES_RATIO = (4000*2.75)/360; //TODO: double check that this is accurate, any inconsistency is important
    public Turret(HardwareMap hardwareMap, double p, double i, double d){
        this.turretServo = hardwareMap.get(Servo.class, "turretServo");
        this.turretEncoder = hardwareMap.get(DcMotor.class, "turretEncoder");
        this.hardwareMap = hardwareMap;
        pidController = new PIDController(p, i, d, new ElapsedTime());
    }
    public void update(){
        pidController.update(getTurretAngle());
        if(isDisabled){return;}
        turretServo.setPosition(((pidController.getPower())/2)+0.5);
    }
    public void setDisabled(boolean disabled){
        isDisabled = disabled;
    }
    public void setTurretAngle(double angle){
        if(isDisabled){
            return;
        }
        pidController.setTarget(angle);
        update();
    }
    public double getTurretAngle(){
        return turretEncoder.getCurrentPosition()* ECODER_TICKS_TO_DEGREES_RATIO;
    }
    public boolean isAtAngle(){
        return Math.abs(getTurretAngle() - pidController.getTarget()) <= errorTolerance;
    }
    public double getTarget(){
        return pidController.getTarget();
    }
}