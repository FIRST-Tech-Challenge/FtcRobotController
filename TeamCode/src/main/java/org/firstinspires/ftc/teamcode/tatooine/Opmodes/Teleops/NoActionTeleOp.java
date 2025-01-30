package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

@TeleOp(name = "NATO", group = "TeleOp")
public class NoActionTeleOp extends LinearOpMode {
    // Determines whether the robot is on the red alliance
    private final boolean isRed = CheckAlliance.isRed();

    // Wrist position variable, initialized at the midpoint
    private double position = 0.5;

    // Sensitivity of wrist position adjustment
    private final double WRIST_SENSITIVITY = 0.03;

    @Override
    public void runOpMode() {
        // Initialize subsystems with required parameters
        Arm arm = new Arm(this, false);
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Camera camera = new Camera(this, false, isRed);

        // Create enhanced gamepad handlers
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);

        waitForStart(); // Wait for start signal

        while (opModeIsActive()) {
            // Drive system - field-oriented control
            drive.fieldDrive(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));

            // Intake control - right trigger for intake, left trigger for outtake
            intake.setPower(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                    gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            // Arm angle and extension control
            arm.setPowerAngleWithF(gamepadEx2.getStick(GamepadKeys.Stick.LEFT_STICK_Y));
            arm.setPowerExtendWithLimits(gamepadEx2.getStick(GamepadKeys.Stick.RIGHT_STICK_Y));

            // Reset IMU when Start button is pressed on gamepad1
            if (gamepadEx1.justPressedButton(GamepadKeys.Button.START)) {
                drive.resetIMU();
            }

            // Wrist position presets based on button presses
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.CROSS)) {
                wrist.home();
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE)) {
                wrist.intake();
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.TRIANGLE)) {
                wrist.straight();
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) {
                wrist.scoreSample();
            }

            // Toggle specimen detection mode in camera
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.START)) {
                camera.setSpecimen(!camera.isSpecimen());
            }

            // Set wrist angle based on camera detection
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP)) {
                wrist.setAngleWristAngle(camera.getAngle());
            }

            // Adjust wrist position incrementally
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                position = MathUtil.wrap(position + WRIST_SENSITIVITY, 0, 1);
                wrist.setPositionWristAngle(position);
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.LEFT_BUMPER)) {
                position = MathUtil.wrap(position - WRIST_SENSITIVITY, 0, 1);
                wrist.setPositionWristAngle(position);
            }

            // Update gamepad states
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);

            telemetry.update(); // Refresh telemetry output
        }
    }
}
