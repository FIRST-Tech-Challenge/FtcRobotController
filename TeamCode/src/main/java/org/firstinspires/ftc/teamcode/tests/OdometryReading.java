package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Odometry Reading Test", group="tests")
public class OdometryReading extends LinearOpMode{

    //for three odometry wheels setup
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = hardwareMap.get(DcMotor.class, "LEncoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "REncoder");
        frontEncoder = hardwareMap.get(DcMotor.class, "FEncoder");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("left encoder: ", leftEncoder.getCurrentPosition());
            telemetry.addData("right encoder: ", rightEncoder.getCurrentPosition());
            telemetry.addData("front encoder: ", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }


}
