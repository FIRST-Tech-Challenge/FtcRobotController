package org.firstinspires.ftc.teamcode.Lansford;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class MathOpModeRL extends OpMode {
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        double rightStickY = gamepad1.right_stick_y;
        boolean bButton = gamepad1.b;/* when the b button is pressed, then it will be true,
         but when it isn't it will be false*/
        double stickDifference = gamepad1.left_stick_y - gamepad1.right_stick_y;
        double triggerSum = gamepad1.left_trigger + gamepad1.right_trigger;

        telemetry.addData("Right stick y", rightStickY); /*this is what tells the code that when
        the right stick is moved, to run this part of a code*/
        telemetry.addData("B Button status", bButton);/* same with this part, but for the b button*/
        telemetry.addData("Stick difference", stickDifference);
        telemetry.addData("Trigger sum", triggerSum);
    }
}
