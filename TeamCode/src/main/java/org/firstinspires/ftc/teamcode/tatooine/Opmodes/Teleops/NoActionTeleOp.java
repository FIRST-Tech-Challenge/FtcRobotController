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
            // Update gamepad states
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
            // Drive system - field-oriented control
            drive.fieldDrive(new Pose2d( - MathUtil.applyDeadzone(gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_X),0.1), MathUtil.applyDeadzone(gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y), 0.1), MathUtil.applyDeadzone(gamepadEx1.getStick(GamepadKeys.Stick.RIGHT_STICK_X),0.1)));

            // Intake control - right trigger for intake, left trigger for outtake
            intake.setPower(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            // Arm angle and extension control

            arm.setPowerAngleWithF(MathUtil.applyDeadzone(gamepadEx2.getStick(GamepadKeys.Stick.LEFT_STICK_Y),0.1));
            arm.setPowerExtend(MathUtil.applyDeadzone(gamepadEx2.getStick(GamepadKeys.Stick.RIGHT_STICK_Y), 0.1));

            // Wrist position presets based on button presses
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE)) {
                wrist.intakeUp();
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.TRIANGLE)) {
                wrist.intakeFlat();
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) {
                wrist.doc();
            }

            // Reset IMU when Start button is pressed on gamepad1
            if (gamepadEx1.justPressedButton(GamepadKeys.Button.START)) {
                drive.resetIMU();
            }
            telemetry.addData("angle", arm.getAngle());
            telemetry.addData("extend", arm.getExtend());
            telemetry.addData("exAmp", arm.getCurrentExtend());
            telemetry.update(); // Refresh telemetry output
        }
    }
}
