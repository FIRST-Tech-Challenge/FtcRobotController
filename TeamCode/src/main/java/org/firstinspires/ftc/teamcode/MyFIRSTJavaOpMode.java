package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class MyFIRSTJavaOpMode extends LinearOpMode {
    private DcMotor motorTest;
    private TouchSensor touchSensor;

    PID PIDTest = new PID(0.1, 0, 0);


    @Override

    public void runOpMode() {

        motorTest = hardwareMap.get(DcMotor.class, "motorTest");

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        telemetry.addData("Status", "Initialized");

        telemetry.update();

// Wait for the game to start (driver presses PLAY)

        waitForStart();

// run until the end of the match (driver presses STOP)

        double tgtPower = 0;
        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.update();
            tgtPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motorTest.getPower());
            tgtPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}