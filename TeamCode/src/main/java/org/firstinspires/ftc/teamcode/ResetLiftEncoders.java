package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Reset LIFT Encoders", group = "Utilities")
public class ResetLiftEncoders extends OpMode {
    Robot robot;

    public void init () {
        robot = new Robot(this, false);
    }

    public void loop () {
        telemetry.addData("Left lift motor encoder: ", robot.lift1.getCurrentPosition());
        telemetry.addData("Right lift motor encoder: ", robot.lift2.getCurrentPosition());
        telemetry.update();

        if (gamepad1.y) {
            robot.resetLiftEncoders();
        }
    }
}
