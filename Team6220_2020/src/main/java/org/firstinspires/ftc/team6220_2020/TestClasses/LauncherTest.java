package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LauncherTest", group = "TeleOp")
public class LauncherTest extends LinearOpMode{

    // Motors
    DcMotor motorLauncher;

    @Override
    public void runOpMode() {

        // Initialize
        motorLauncher = hardwareMap.dcMotor.get("motorLauncher");
        motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // If/Else statement to detect whether or not the "a" button is pressed on the controller.
            // Determines whether the motor is running or stopped.
            if (gamepad1.a) {
                motorLauncher.setPower(1.0);
            }

            else {
                motorLauncher.setPower(0.0);
            }

            // Telemetry
            telemetry.addData("Launcher Motor Power: ", motorLauncher.getPower());
            telemetry.update();

            idle();
        }
    }
}