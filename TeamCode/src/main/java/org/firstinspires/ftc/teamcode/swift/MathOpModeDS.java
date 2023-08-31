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
        double speedSideways = -gamepad1.right_stick_y / 2.0;
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("speed Sideways", speedSideways);

        if(gamepad1.a)
            telemetry.addData("A Button", "pressed");
        if(gamepad1.b)
            telemetry.addData("B Button", "pressed");

        double YDiffrence = gamepad1.left_stick_y - gamepad1.right_stick_y;
        telemetry.addData("Y stick difference", YDiffrence);

        double TriggerDiff = gamepad1.left_trigger - gamepad1.right_trigger;
        telemetry.addData("Trigger difference",TriggerDiff );

        /*this is whats being called to show on the drive hub, whats showing are the values that
        control/display the different aspects of you controllers movement*/
    }
}
//ends code (code stops parsing)