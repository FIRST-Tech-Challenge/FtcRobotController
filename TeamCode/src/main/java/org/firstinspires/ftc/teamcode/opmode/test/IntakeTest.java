package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Intake;

@Config
@TeleOp(name = "IntakeTest", group = "Test")

public class IntakeTest extends LinearOpMode {

    private Intake intake;

    public static boolean loggingOn = false;

    @Override
    public void runOpMode() {

        intake = new Intake(hardwareMap, telemetry, true);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                intake.up();
            } else if (gamepad1.left_bumper) {
                intake.down();
            }

            if (gamepad1.a)
                intake.forward();
            else if (gamepad1.x)
            {
                intake.stop();
            }

            intake.update();
        }
    }
}
