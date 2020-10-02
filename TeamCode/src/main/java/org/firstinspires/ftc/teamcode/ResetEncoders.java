package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//INSTRUCTIONS:
// align modules to be facing the same direction (make sure not 180 degrees apart)
// press y button on controller 1 (configured with start+A)
// make sure telemetry encoder values are both 0
// if robot controls are inverted, repeat this process, but turn both modules 180 degrees away from where you reset them before

//MUST be run every time program is downloaded
//does NOT have to be run before every TeleOp/Auto run
//probably should be used to verify that encoders have not drifted before every competition match

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Reset Encoders", group = "Utilities")
public class ResetEncoders extends OpMode {
    Robot robot;

    public void init () {
        robot = new Robot(this, false);
    }

    public void loop () {
        telemetry.addData("LEFT Module Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation().getAngle());
        telemetry.addData("RIGHT Module Orientation: ", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
        telemetry.update();

        if (gamepad1.y) {
            robot.driveController.resetEncoders();
        }
    }
}
