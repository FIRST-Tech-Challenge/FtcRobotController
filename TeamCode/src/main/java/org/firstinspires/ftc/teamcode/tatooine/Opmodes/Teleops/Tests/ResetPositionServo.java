package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
@Disabled
@TeleOp
public class ResetPositionServo extends LinearOpMode{

    public void runOpMode() {
        waitForStart();
        Wrist wrist = new Wrist(this, false);
        while (opModeIsActive()) {
            if (gamepad1.cross) {
                wrist.setPosition(1);
            }
            if (gamepad1.circle){
                wrist.setPosition(0);
            }
        }
        }
    }