package org.firstinspires.ftc.teamcode.swift;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//imports package (teamcode) to code
@TeleOp()
public class MathOpModeDS extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Made By","Dawston");
        //digital signature so I know its mine
    }

    @Override
    public void loop() {

        double speedForward = -gamepad1.left_stick_y / 2.0;
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("speed Forward", speedForward);
        /*this is whats being called to show on the drive hub, whats showing are the values that
        control/display the different aspects of you controllers movement*/
    }
}
//ends code (code stops parsing)