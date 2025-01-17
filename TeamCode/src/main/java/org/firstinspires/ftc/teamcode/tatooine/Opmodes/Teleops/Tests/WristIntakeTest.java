package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
@Disabled
@TeleOp
public class WristIntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Wrist wrist = new Wrist(this, true);
        Intake intake = new Intake(this, false, true);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.cross) {
                wrist.changeState();
            }
            if (gamepad1.circle) {
                intake.setPowerFun(1);
            }
            if (gamepad1.square) {
//                wrist.setPower(1);
            }
        }

    }
    }
