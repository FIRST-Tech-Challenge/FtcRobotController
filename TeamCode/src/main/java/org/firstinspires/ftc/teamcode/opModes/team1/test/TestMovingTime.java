package org.firstinspires.ftc.teamcode.opModes.team1.test;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.components.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.HDriveWrapper;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.motors.TrapezoidalProfile;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.motors.TrapezoidalProfileByTime;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name = "Test Moving Time", group = "Test")
public class TestMovingTime extends TeleOpLinearModeBase {
    private ElapsedTime runtime = new ElapsedTime();
    private HDriveWrapper drive;
    private IMU imu;

    private double duration = 3.0;
    private boolean moveSideways = false;
    private boolean moveBackwards = false;

    AprilTagDetection tagOfInterest = null;

    public void run() {
        LinearOpMode opMode = this; // For quick copying-and-pasting with generic opmode

        // Set up auto driving
        imu = HardwareMapContainer.getMap().get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        imu.resetYaw();

        // Get the third motor as a spinner motor
        drive = new HDriveWrapper(new HDrive(
                HardwareMapContainer.motor0,
                HardwareMapContainer.motor1,
                HardwareMapContainer.motor3,
                0,
                Math.PI,
                Math.PI/2
        ), imu);

        Inputs.gamepad1.getGamepadButton(PSButtons.TRIANGLE).whenPressed(new InstantCommand(() -> {
            duration += 0.1;
            duration %= 10.0; // Max 10.0, min 0
        }));
        Inputs.gamepad1.getGamepadButton(PSButtons.CROSS).whenPressed(new InstantCommand(() -> {
            duration -= 0.1;
            duration += 10.0;
            duration %= 10.0; // Max 10.0, min 0
        }));
        Inputs.gamepad1.getGamepadButton(PSButtons.SQUARE).whenPressed(new InstantCommand(() -> {
            if(!moveBackwards) {
                moveBackwards = true;
            } else {
                moveBackwards = false;
                moveSideways = !moveSideways;
            }
        }));
        Inputs.gamepad1.getGamepadButton(PSButtons.CIRCLE).whenPressed(new InstantCommand(() -> {
            driveByTime(duration, moveSideways, moveBackwards, opMode, TelemetryContainer.getTelemetry());
        }));
        Inputs.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {

        }));

        opMode.waitForStart();
        while(!opMode.isStopRequested() && opMode.opModeIsActive()) {
            // Telemetry logs
            TelemetryContainer.getTelemetry().addData("Elapsed", runtime.seconds());
            if(moveSideways) {
                if(moveBackwards) {
                    TelemetryContainer.getTelemetry().addLine("Moving RIGHT     [Toggle: Square]");
                } else {
                    TelemetryContainer.getTelemetry().addLine("Moving LEFT      [Toggle: Square]");
                }
            } else {
                if(moveBackwards) {
                    TelemetryContainer.getTelemetry().addLine("Moving BACKWARDS [Toggle: Square]");
                } else {
                    TelemetryContainer.getTelemetry().addLine("Moving FORWARDS  [Toggle: Square]");
                }
            }
            TelemetryContainer.getTelemetry().addLine(String.format("Move Duration = %.2f seconds [Increase: Triangle; Decrease: Cross]", duration));
            TelemetryContainer.getTelemetry().addLine("[Start Moving: Circle]");

            CommandScheduler.getInstance().run();
            TelemetryContainer.getTelemetry().update();
        }
        opMode.sleep(200);
    }

    void driveByTime(double timeSeconds, boolean moveSideways, boolean moveBackwards, LinearOpMode opMode, MultipleTelemetry telemetry) {
        runtime.reset();
        int initialCounts = HDriveWrapper.getTotalEncoderCounts(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor3);

        TrapezoidalProfileByTime profile = new TrapezoidalProfileByTime(timeSeconds, new TrapezoidalProfile(0.7, 1));

        double speedMultiplier = moveBackwards ? -1 : 1;

        while(!opMode.isStopRequested() && opMode.opModeIsActive() && (runtime.seconds() < timeSeconds)) {
            telemetry.addData("Driving Time", runtime.seconds());
            telemetry.addData("Driving Counts", HDriveWrapper.getTotalEncoderCounts(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor3));
            telemetry.addData("Driving Distance (inches)", DriveConstants.encoderTicksToInches((double)(HDriveWrapper.getTotalEncoderCounts(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor3) - initialCounts) / 2)); // /2 as 2 motors
            if(moveSideways) { // Relative to original heading
                drive.fieldOrientedDriveAbsoluteRotation(0, speedMultiplier * profile.predict(runtime.seconds())); // So /``\ in velocity (limit acceleration)
            } else {
                drive.fieldOrientedDriveAbsoluteRotation(speedMultiplier * profile.predict(runtime.seconds()), 0); // So /``\ in velocity (limit acceleration)
            }
            telemetry.update();
            opMode.sleep(50);
        }
        HardwareMapContainer.motor0.set(0);
        HardwareMapContainer.motor1.set(0);
        HardwareMapContainer.motor3.set(0);
    }
}