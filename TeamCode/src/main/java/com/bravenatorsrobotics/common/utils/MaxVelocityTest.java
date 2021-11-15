package com.bravenatorsrobotics.common.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Max Velocity", group="Util Opmode")
public class MaxVelocityTest extends LinearOpMode {

    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override

    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        motor = hardwareMap.get(DcMotorEx.class, "fl");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addLine("Motor labeled fl on start will rotate at max power.");
        telemetry.addLine("Disclaimer: Make sure that the battery is charged");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        motor.setPower(1);

        while (opModeIsActive()) {

            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Maximum Velocity", maxVelocity);
            telemetry.update();

        }

    }

}