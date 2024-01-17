package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Template {
    private DcMotor motor;
    private DcMotor[] motors;
    private Servo servo;
    private CRServo crServo;
    private IMU imu;
    private IMU.Parameters parameters;

    public Template(DcMotor motor, DcMotor[] motors, Servo servo, CRServo crServo, IMU imu) {
        this.motor = motor;
        this.motors = motors;
        this.servo = servo;
        this.crServo = crServo;
        this.imu = imu;

        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
    }

    public void reset() {imu.resetYaw();}

    public void something(double power) {
        motor.setPower(power);
    }
}
