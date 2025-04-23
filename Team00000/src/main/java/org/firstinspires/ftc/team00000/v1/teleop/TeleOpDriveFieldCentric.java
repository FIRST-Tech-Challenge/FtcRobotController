package org.firstinspires.ftc.team00000.v1.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team00000.v1.RobotHardware;
@Disabled
@TeleOp(name = "Field Centric", group = "Robot")

public class TeleOpDriveFieldCentric extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;
        double verticalArm;
        double horizontalArm;

        // Initialize all the hardware, using the hardware class.
        robot.init();

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP).
        while (opModeIsActive()) {

            // Field Centric Mode use the left joystick to go forward & strafe and the right joystick to rotate from
            // the perspective of the driver
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            turn = gamepad1.right_stick_x;

            // Combine drive, strafe, and turn for blended motion. Use RobotHardware class
            robot.driveFieldCentric(drive, strafe, turn);

            verticalArm = gamepad1.right_trigger;
            horizontalArm = gamepad1.left_trigger;

            if (gamepad1.right_trigger == 1 && gamepad1.left_trigger == 0) {
                if (gamepad1.a) {
                    robot.setVerticalClawPosition(robot.CLAW_OPEN);
                } else {
                    robot.setVerticalClawPosition(robot.CLAW_CLOSE);
                }
                robot.setHorizontalClawPosition(robot.CLAW_OPEN);
                robot.setVerticalWristPosition(robot.VERTICAL_WRIST_RUNG);
                robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
                robot.setVerticalArmPosition(robot.VERTICAL_ARM_HIGH_BASKET);
            } else if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger == 0) {
                if (gamepad1.a) {
                    robot.setVerticalClawPosition(robot.CLAW_OPEN);
                } else {
                    robot.setVerticalClawPosition(robot.CLAW_CLOSE);
                }
                robot.setHorizontalClawPosition(robot.CLAW_OPEN);
                robot.setVerticalWristPosition(robot.VERTICAL_WRIST_RUNG);
                robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
                robot.setVerticalArmPosition(robot.VERTICAL_ARM_LOW_BASKET);
            } else if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
                robot.setVerticalClawPosition(robot.CLAW_CLOSE);
                robot.setHorizontalClawPosition(robot.CLAW_OPEN);
                robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
                robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
                robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
            } else if (gamepad1.right_trigger == 0 && gamepad1.left_trigger > 0) {
                robot.setVerticalClawPosition(robot.CLAW_OPEN);
                if(gamepad1.a) {
                    robot.setHorizontalClawPosition(robot.CLAW_OPEN);
                } else {
                    robot.setHorizontalClawPosition(robot.CLAW_CLOSE);
                }
                robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
                robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_PICKUP);
                robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
            } else if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
                robot.setVerticalClawPosition(robot.CLAW_OPEN);
                robot.setHorizontalClawPosition(robot.CLAW_CLOSE);

                if(gamepad1.x) {
                    robot.setVerticalWristPosition(robot.VERTICAL_WRIST_RUNG);
                } else if (gamepad1.y) {
                    robot.setVerticalWristPosition(robot.VERTICAL_WRIST_BUCKET);
                } else if(gamepad1.b) {
                    robot.setVerticalWristPosition(robot.VERTICAL_WRIST_DROP);
                } else {
                    robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
                }

                robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
                robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
            }
            robot.setHorizontalArmPosition(horizontalArm);

            // Send a telemetry message to explain controls and show robot status
            telemetry.addData("\nStatus", "Run Time: " + runtime);
            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.addData("\nVertical Arm Power", "%.2f", verticalArm);
            telemetry.addData("Horizontal Arm", "%.2f", horizontalArm);
            telemetry.update();
        }
    }
}
