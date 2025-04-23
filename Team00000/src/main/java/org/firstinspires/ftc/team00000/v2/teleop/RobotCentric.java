package org.firstinspires.ftc.team00000.v2.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.v2.RobotHardware;
@Disabled
@TeleOp(name = "Robot Centric", group = "TeleOp")

public class RobotCentric extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;
        double shoulder;
        double arm;

        // Initialize all the hardware, using the hardware class.
        robot.init();

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP).
        while (opModeIsActive()) {

            // Field Centric Mode use the left joystick to go forward & strafe and the right joystick to rotate from
            // the perspective of the driver
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            turn = gamepad1.right_stick_x;

            arm = gamepad1.right_trigger;
            shoulder = gamepad1.left_trigger;

            // Combine drive, strafe, and turn for blended motion. Use RobotHardware class
            robot.driveRobotCentric(drive, strafe, turn);
            robot.setArmPosition(4 * arm * robot.ARM_TICKS_PER_REV);

            if (gamepad1.right_bumper) {
                robot.setClawPosition(robot.CLAW_CLOSE);
            } else if (gamepad1.left_bumper) {
                robot.setClawPosition(robot.CLAW_OPEN);
            }

            if (gamepad1.right_stick_button) {
                robot.setWristPosition(robot.WRIST_ROTATE);
            } else if (gamepad1.left_stick_button) {
                robot.setWristPosition(robot.WRIST_STRAIGHT);
            }

            if (gamepad1.x) {
                robot.setShoulderPosition(robot.SHOULDER_LOW_BUCKET);
            } else if (gamepad1.b) {
                robot.setShoulderPosition(robot.SHOULDER_HIGH_BUCKET);
            } else if (gamepad1.dpad_left) {
                robot.setShoulderPosition(robot.SHOULDER_LOW_CHAMBER);
            } else if (gamepad1.dpad_right) {
                robot.setShoulderPosition(robot.SHOULDER_HIGH_CHAMBER);
            } else if (gamepad1.dpad_up) {
                robot.setShoulderPosition(robot.SHOULDER_ATTACH_HANGING_HOOK);
            } else if(gamepad1.dpad_down) {
                robot.setShoulderPosition(robot.SHOULDER_WINCH_ROBOT);
            } else if (gamepad1.left_trigger > 0) {
                robot.setShoulderPosition((23 * robot.SHOULDER_TICKS_PER_DEGREE
                        - shoulder * 14 * robot.SHOULDER_TICKS_PER_DEGREE));
            }

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.update();
        }
    }
}
