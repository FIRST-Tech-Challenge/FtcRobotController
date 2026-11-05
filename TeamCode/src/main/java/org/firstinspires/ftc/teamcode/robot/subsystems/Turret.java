package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.broombots.sweep.Classes.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.PIDController;

public class Turret implements Subsystem {
    private Servo turretServo;
    private DcMotor turretEncoder;
    private PIDController pidController;
    private boolean isDisabled = false;
    public Turret(HardwareMap hardwareMap, double p, double i, double d, String servo, String encoder){
        turretServo = hardwareMap.get(Servo.class, servo);
        turretEncoder = hardwareMap.get(DcMotor.class, encoder);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidController = new PIDController(p, i, d, new ElapsedTime());
    }
    public void setDisabled(boolean disabled){
;       if(disabled){turretServo.setPosition(0.5);}
        isDisabled = disabled;

    }
    public void update(){
        pidController.update(getTurretAngle());
        if(isDisabled){return;}
        turretServo.setPosition(((pidController.getPower())/2)+0.5);
    }
    public void setTurretAngle(double angle){
        pidController.setTarget(angle);
    }
    public double getTurretAngle(){
        //Calculation is 4000 encoder ticks to a rotation of the turret, a rotation of the turret is 2.75 rotations of the servo, and dividing by 360 gets it into degrees
        final double ECODER_TICKS_TO_DEGREES_RATIO = (4000*2.75)/360; //TODO: double check that this is accurate, any inconsistency is important
        return turretEncoder.getCurrentPosition()* ECODER_TICKS_TO_DEGREES_RATIO;
    }
    public boolean isAtAngle(){
        double errorTolerance = 2; //TODO: find a good error tolerance, this is just a guess
        return Math.abs(getTurretAngle() - pidController.getTarget()) <= errorTolerance;
    }
    public double getTarget(){
        return pidController.getTarget();
    }
}