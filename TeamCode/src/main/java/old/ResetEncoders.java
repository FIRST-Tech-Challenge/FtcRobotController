package old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//INSTRUCTIONS:
// align modules to be facing the same direction (make sure not 180 degrees apart)
// press y button on controller 1 (configured with start+A)
// make sure telemetry encoder values are both 0
// if robot controls are inverted, repeat this process, but turn both modules 180 degrees away from where you reset them before

//MUST be run every time program is downloaded
//does NOT have to be run before every TeleOp/Auto run
//probably should be used to verify that encoders have not drifted before every competition match

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Reset Encoders", group = "Utilities")
public class ResetEncoders extends LinearOpMode {
    Robot robot;

    public void runOpMode () {
        robot = new Robot(this, false);

        waitForStart();
        robot.updateBulkData();

        robot.driveController.resetEncoders();
        telemetry.addData("LEFT Module Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation().getAngle());
        telemetry.addData("RIGHT Module Orientation: ", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
        telemetry.update();
    }
}
