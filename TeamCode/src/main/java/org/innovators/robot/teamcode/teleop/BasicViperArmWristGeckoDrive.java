package org.innovators.robot.teamcode.teleop;

import static org.innovators.robot.teamcode.util.Constants.SERVO_POWER_DOWN;
import static org.innovators.robot.teamcode.util.Constants.SERVO_POWER_MID;
import static org.innovators.robot.teamcode.util.Constants.SERVO_POWER_UP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.innovators.robot.teamcode.hardware.RobotHardware;
import org.innovators.robot.teamcode.util.Constants;

@TeleOp(name="Viper Arm Wrist Gecko Wheel Control", group="TeleOp")
public class BasicViperArmWristGeckoDrive extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Reset the runtime
        runtime.reset();

        waitForStart();

        while (opModeIsActive()) {
            // Control the drive power
            double drive = -gamepad1.left_stick_y * Constants.DRIVE_SPEED;
            double turn  =  gamepad1.right_stick_x * Constants.TURN_SPEED;
            double leftDrivePower = drive + turn;
            double rightDrivePower = drive - turn;

            robot.leftDriveMotor.setPower(leftDrivePower);
            robot.rightDriveMotor.setPower(rightDrivePower);

            // Control the arm extension with gamepad triggers
            double armExtensionPower = gamepad1.right_trigger - gamepad1.left_trigger;
            robot.armExtensionMotor.setPower(armExtensionPower);

            // Control the arm wrist rotation with gamepad buttons
            if (gamepad1.a) {
                // Move arm to pick up position
                robot.armWristTorqueServo.setPosition(SERVO_POWER_UP);
            } else if (gamepad1.b) {
                // Move arm to drop position
                robot.armWristTorqueServo.setPosition(SERVO_POWER_DOWN);
            } else {
                robot.armWristTorqueServo.setPosition(SERVO_POWER_MID);
            }

            // Control the gecko wheel with gamepad bumpers
            if (gamepad1.right_bumper) {
                robot.geckoWheelSpeedServo.setPosition(SERVO_POWER_DOWN);
            } else if (gamepad1.left_bumper) {
                robot.geckoWheelSpeedServo.setPosition(SERVO_POWER_UP);
            } else {
                robot.armWristTorqueServo.setPosition(SERVO_POWER_DOWN);
            }

            // Telemetry for debugging
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Left Drive Motor Power", leftDrivePower);
            telemetry.addData("Right Drive Motor Power", rightDrivePower);
            telemetry.addData("Extension Motor Power", armExtensionPower);
            telemetry.addData("Extension Motor Position", robot.armExtensionMotor.getCurrentPosition());
            telemetry.addData("Arm Wrist Servo Position", robot.armWristTorqueServo.getPosition());
            telemetry.addData("Gecko Wheel Servo Position", robot.geckoWheelSpeedServo.getPosition());
            telemetry.update();
        }
    }
}

