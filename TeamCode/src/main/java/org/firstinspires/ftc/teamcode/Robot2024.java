package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.PublicKey;

public class Robot2024 {
    Servo gripper;
    DcMotor Shoulder_Motor;

    double gripper_open_position = 0.0;
    double gripper_close_position = 0.3;
    double drivePowerModifier = 0.5;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private double strafePower;
    private double forwardPower;
    private double turnPower;

    public Robot2024(Servo gripper, DcMotor Shoulder_Motor) {
        this.gripper = gripper;
        this.Shoulder_Motor = Shoulder_Motor;
    }

    public Robot2024(HardwareMap hardwareMap) {
        this.Shoulder_Motor = hardwareMap.dcMotor.get("Shoulder_Motor");
        this.Shoulder_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.Shoulder_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.gripper = hardwareMap.servo.get("Gripper_Servo");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void closeGripper(){
        gripper.setPosition(this.gripper_close_position);
    }

    public void openGripper(){
        gripper.setPosition(this.gripper_close_position);
    }

    public void setShoulderPower(double powerLevel){
        Shoulder_Motor.setPower(powerLevel);
    }

    public void setStrafePower(double powerLevel) {
        strafePower = powerLevel;
        this.setDrivePowers();
    }

    public void setForwardPower(double powerLevel) {
        forwardPower = powerLevel;
        this.setDrivePowers();
    }

    public void setTurnPower(double powerLevel) {
        turnPower = powerLevel;
        this.setDrivePowers();
    }

    private void setDrivePowers() {
        rightFront.setPower((-strafePower - forwardPower + turnPower) * drivePowerModifier);
        leftFront.setPower((strafePower - forwardPower - turnPower) * drivePowerModifier);
        leftBack.setPower((-strafePower - forwardPower - turnPower) * drivePowerModifier);
        rightBack.setPower((strafePower - forwardPower + turnPower) * drivePowerModifier);

    }

}
