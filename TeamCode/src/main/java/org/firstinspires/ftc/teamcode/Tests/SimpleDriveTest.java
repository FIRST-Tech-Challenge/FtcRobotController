package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.Chassis.ChassisDriver;
@TeleOp(name = "SimpleDriveTest")
public class SimpleDriveTest extends LinearOpMode {
    DcMotorEx lb, lf, rb, rf;

    @Override
    public void runOpMode() throws InterruptedException {
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        ChassisDriver.initializeMotors(lf, rf, lb, rb);
        ChassisDriver.resetWheelEncoders(lf, rf, lb, rb);

        waitForStart();

        while (opModeIsActive()) {

            double forwardPower = -gamepad1.left_stick_y * 2;
            double rotationPower = -gamepad1.right_stick_x * 4;
            double leftPower = -gamepad1.left_stick_x * 2;

            telemetry.addData("forward: ", forwardPower);
            telemetry.addData("rotation: ", rotationPower);
            telemetry.addData("left power: ", leftPower);
            telemetry.update();

            ChassisDriver.rawDrive(forwardPower, leftPower, rotationPower);
        }
    }
}