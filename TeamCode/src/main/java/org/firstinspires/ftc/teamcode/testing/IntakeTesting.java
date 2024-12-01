package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake;

@TeleOp(name = "IntakeTest", group = "test")
public class IntakeTesting extends LinearOpMode {

    Intake intake;
    boolean intakeLiftActive;


    @Override
    public void runOpMode() throws InterruptedException {

        intakeLiftActive = false;
        intake = new Intake(this);
        intake.liftIntake();

        waitForStart();

        while (opModeIsActive()) {
            intake.collect(gamepad1.left_trigger);

            if (gamepad1.right_bumper && !intakeLiftActive) {
                intakeLiftActive = true;
                intake.liftIntake();

            } else {
                intakeLiftActive = false;
            }
        }
    }
}
