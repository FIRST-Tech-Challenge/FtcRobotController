package org.firstinspires.ftc.teamcode.swift;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//imports package (teamcode) to code
@TeleOp()
@Disabled
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
// the individual values is what will pop up here when you move the joysticks


        boolean Bbutton = (gamepad1.b);
// make a boolean variable that responds as true when b button is pressed
        telemetry.addData("B button", Bbutton);
// send that information to display on the driver hub
        double YDifference = gamepad1.left_stick_y - gamepad1.right_stick_y;
        telemetry.addData("Y stick difference", YDifference);
// difference of the stick values when moved
        double TriggerSum = gamepad1.left_trigger + gamepad1.right_trigger;
        telemetry.addData("Trigger Sum",TriggerSum );
// sum of both the trigger values
        /*this is whats being called to show on the drive hub, whats showing are the values that
        control/display the different aspects of you controllers movement*/
    }
}
//ends code (code stops parsing)