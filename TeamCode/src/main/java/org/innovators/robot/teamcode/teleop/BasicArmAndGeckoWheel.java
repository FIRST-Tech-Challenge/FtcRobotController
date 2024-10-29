package org.innovators.robot.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.innovators.robot.teamcode.hardware.RobotHardware;
import org.innovators.robot.teamcode.util.Constants;

@TeleOp(name="Arm and Gecko Wheel Control", group="TeleOp")
public class BasicArmAndGeckoWheel extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Reset the runtime
        runtime.reset();

        waitForStart();

        while (opModeIsActive()) {
            // Control the arm rotation with gamepad buttons
            if (gamepad1.a) {
                // Move arm to pick up position
                robot.armServo.setPosition(1.0);
            } else if (gamepad1.b) {
                // Move arm to drop position
                robot.armServo.setPosition(0.0);
            }

            // Control the arm extension with gamepad triggers
            double extensionPower = gamepad1.right_trigger - gamepad1.left_trigger;
            robot.armExtensionMotor.setPower(extensionPower);

            // Control the gecko wheel with gamepad bumpers
            if (gamepad1.right_bumper) {
                robot.geckoWheelMotor.setPower(Constants.GECKO_WHEEL_POWER);
            } else if (gamepad1.left_bumper) {
                robot.geckoWheelMotor.setPower(-Constants.GECKO_WHEEL_POWER);
            } else {
                robot.geckoWheelMotor.setPower(0);
            }

            // Telemetry for debugging
            telemetry.addData("Servo Position", robot.armServo.getPosition());
            telemetry.addData("Extension Power", extensionPower);
            telemetry.addData("Extension Position", robot.armExtensionMotor.getCurrentPosition());
            telemetry.addData("Gecko Wheel Power", robot.geckoWheelMotor.getPower());
            telemetry.update();
        }
    }
}

