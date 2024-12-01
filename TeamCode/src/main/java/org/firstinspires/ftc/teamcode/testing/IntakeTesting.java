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

        intake = new Intake(this);
        intake.initIntake();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.circle){
                intake.collect(1);

            }
            else if (gamepad2.square){
                intake.collect(-1);
            }

            }
        }
    }

