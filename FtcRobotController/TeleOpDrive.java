package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tank", group = "Robot")

public class TeleOpDrive extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double left;
        double right;
        double shoulder;
        double arm;

        // Initialize all the hardware, using the hardware class.
        robot.init();

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP).
        while (opModeIsActive()) {
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            left = Range.clip(drive + turn, -1.0, 1.0) ;
            right = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // left = -gamepad1.left_stick_y ;
            // right = -gamepad1.right_stick_y ;

            arm = gamepad1.right_trigger;
            shoulder = gamepad1.left_trigger;
            robot.setDrivePower(left, right);
            robot.setArmPosition(arm);

            if (gamepad1.right_bumper) {
                robot.setClawPosition(robot.CLAW_CLOSE);
            } else if (gamepad1.left_bumper) {
                robot.setClawPosition(robot.CLAW_OPEN);
            }

            if (gamepad1.right_stick_button) {
                robot.setWristPosition(robot.WRIST_ROTATE);
            } else if (gamepad1.left_stick_button) {
                robot.setWristPosition(robot.WRIST_STRAIGHT);
            } else if (gamepad1.x) {
                robot.setShoulderPosition(robot.SHOULDER_LOW_BUCKET);
            } else if (gamepad1.b) {
                robot.setShoulderPosition(robot.SHOULDER_HIGH_BUCKET);
            } else if (gamepad1.dpad_left) {
                robot.setShoulderPosition(robot.SHOULDER_LOW_RUNG);
            } else if (gamepad1.dpad_right) {
                robot.setShoulderPosition(robot.SHOULDER_HIGH_RUNG);
            } else if (gamepad1.dpad_up) {
                robot.setShoulderPosition(robot.SHOULDER_ATTACH_HANGING_HOOK);
            } else if(gamepad1.dpad_down) {
                robot.setShoulderPosition(robot.SHOULDER_WINCH_ROBOT);
            } else if (gamepad1.left_trigger > 0) {
                robot.setShoulderPosition((22 * robot.SHOULDER_TICKS_PER_DEGREE
                        - shoulder * 11 * robot.SHOULDER_TICKS_PER_DEGREE));
            }

            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
        }
    }
}
