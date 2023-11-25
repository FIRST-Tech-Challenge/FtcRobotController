package org.firstinspires.ftc.teamcode.TestCode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Odometry Test V1")
public class _2023_11_17_01_OdometryTest_V1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor odometryPod1 = hardwareMap.dcMotor.get("motorFL");
        odometryPod1.setDirection(DcMotorSimple.Direction.REVERSE);
        odometryPod1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometryPod1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Odometry Pod Encoder Value: ", odometryPod1.getCurrentPosition());
            telemetry.update();
        }
    }
}