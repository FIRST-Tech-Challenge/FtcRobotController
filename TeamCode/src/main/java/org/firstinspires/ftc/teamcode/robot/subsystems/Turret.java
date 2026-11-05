package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

public class Turret {
    private PIDController pidController;
    private HardwareMap hardwareMap;
    private ServoPlus turretServo;
    private DcMotor turretEncoder;
    private double errorTolerance; //TODO: find a good error tolerance
    private final double ECODER_TICKS_TO_DEGREES_RATIO = (4000*2.75)/360; //TODO: double check that this is accurate, any inconsistency is important
    private double target = 0;
    public Turret(PIDController pidController, HardwareMap hardwareMap){
        this.pidController = pidController;
        this.turretServo = hardwareMap.get(ServoPlus.class, "turretServo");
        this.turretEncoder = hardwareMap.get(DcMotor.class, "turretEncoder");
        this.hardwareMap = hardwareMap;
    }
    public void setTurretAngle(double angle){
        target = angle;
        //TODO: make it set the turret angle
    }
    public double getTurretAngle(){
        return turretEncoder.getCurrentPosition()* ECODER_TICKS_TO_DEGREES_RATIO;
    }
    public boolean isAtAngle(){
        return Math.abs(getTurretAngle() - target) <= errorTolerance;
    }

}