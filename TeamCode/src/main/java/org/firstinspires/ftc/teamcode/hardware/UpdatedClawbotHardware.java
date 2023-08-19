package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Allenn23825 on 1/24/2023.
 */
public class UpdatedClawbotHardware {
    public static final double CLAW_OPEN_POSITION = 0.2;
    public static final double CLAW_CLOSED_POSITION = 0.48;
    public DcMotorEx leftDrive;
    public DcMotorEx rightDrive;
    public DcMotorEx arm;
    public Servo claw;
    public AnalogInput potentiometer;
    public enum ArmState {GROUND, LOW, MEDIUM, HIGH}
    public enum ClawState {OPEN, CLOSED}



    public void init(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        arm.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(0);
        potentiometer = hardwareMap.analogInput.get("potentiometer");
    }
}