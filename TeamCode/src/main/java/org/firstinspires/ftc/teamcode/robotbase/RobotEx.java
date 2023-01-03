package org.firstinspires.ftc.teamcode.robotbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;


public class RobotEx {
    // enum to specify opmode type
    public enum OpModeType {
        TELEOP, AUTO
    }

    protected OpModeType opModeType;
    protected Telemetry telemetry;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;

    protected GamepadEx driverOp;
    protected GamepadEx toolOp;

    protected MecanumDriveSubsystem drive = null;
    protected MecanumDriveCommand driveCommand = null;

    protected SampleMecanumDrive rrDrive = null;

    protected IMUSubsystem gyro;
    public Camera camera;

    protected HeadingControllerSubsystem gyroFollow;
    protected HeadingControllerSubsystem cameraFollow;

    public RobotEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                   GamepadEx toolOp) {
        this(hardwareMap, telemetry, driverOp, toolOp, OpModeType.TELEOP, true);
    }

    public RobotEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                   GamepadEx toolOp, OpModeType type) {
        this(hardwareMap, telemetry, driverOp, toolOp, type, true);
    }

    public RobotEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                   GamepadEx toolOp, OpModeType type, Boolean useCameraFollower) {
        if (type == OpModeType.TELEOP) {
            initTele(hardwareMap, telemetry, driverOp, toolOp, useCameraFollower);
            opModeType = OpModeType.TELEOP;
        } else {
            initAuto(hardwareMap, telemetry);
            opModeType = OpModeType.AUTO;
        }
    }

    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        /////////////////////////////////////// FTC Dashboard //////////////////////////////////////
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        this.telemetry = telemetry;

        //////////////////////////////////////////// IMU ///////////////////////////////////////////
        gyro = new IMUSubsystem(hardwareMap, this.telemetry, dashboardTelemetry);
        CommandScheduler.getInstance().registerSubsystem(gyro);

        ////////////////////////////////////////// Camera //////////////////////////////////////////
        camera = new Camera(hardwareMap, dashboard, telemetry,
                () -> this.driverOp.getButton(GamepadKeys.Button.BACK));

        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);

        ////////////////////////// Setup and Initialize Mechanisms Objects /////////////////////////
        initMechanismsAutonomous(hardwareMap);
    }

    public void initTele(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                         GamepadEx toolOp, Boolean useCameraFollower) {
        ///////////////////////////////////////// Gamepads /////////////////////////////////////////
        this.driverOp = driverOp;
        this.toolOp = toolOp;

        /////////////////////////////////////// FTC Dashboard //////////////////////////////////////
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        this.telemetry = telemetry;

        //////////////////////////////////////////// IMU ///////////////////////////////////////////
        gyro = new IMUSubsystem(hardwareMap, this.telemetry, dashboardTelemetry);
        CommandScheduler.getInstance().registerSubsystem(gyro);

        ////////////////////////////////////////// Camera //////////////////////////////////////////
        camera = new Camera(hardwareMap, dashboard, telemetry,
                () -> this.driverOp.getButton(GamepadKeys.Button.BACK));

        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
        drive = new MecanumDriveSubsystem(hardwareMap);
        driveCommand = new MecanumDriveCommand(drive, driverOp::getLeftY, driverOp::getLeftX,
                this::drivetrainTurn, gyro::getRawValue,
                () -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        CommandScheduler.getInstance().registerSubsystem(drive);
        drive.setDefaultCommand(driveCommand);

        /////////////////////////////////////// Gyro Follower //////////////////////////////////////
        gyroFollow = new HeadingControllerSubsystem(gyro::getValue,
                gyro::findClosestOrientationTarget);
        new Trigger(() -> driverOp.getRightY() >= 0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(0), gyroFollow));
        new Trigger(() -> driverOp.getRightY() <= -0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(180), gyroFollow));
        new Trigger(() -> driverOp.getRightX() >= 0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(90), gyroFollow));
        new Trigger(() -> driverOp.getRightX() <= -0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(-90), gyroFollow));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(0)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(180)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(-90)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(90)));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(gyroFollow::toggleState, gyroFollow));

        ////////////////////////////////////// Camera Follower /////////////////////////////////////
//        cameraFollow = new HeadingControllerSubsystem(camera::getObject_x);
        if (useCameraFollower)
            driverOp.getGamepadButton(GamepadKeys.Button.START)
                    .whenPressed(new InstantCommand(cameraFollow::toggleState, cameraFollow));

        ////////////////////////// Setup and Initialize Mechanisms Objects /////////////////////////
        initMechanismsTeleOp(hardwareMap);
    }

    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        // should be overridden by child class
    }

    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        // should be overridden by child class
    }

    public double drivetrainTurn() {
        if (gyroFollow.isEnabled())
            return gyroFollow.calculateTurn();
        if (cameraFollow.isEnabled())
            return cameraFollow.calculateTurn();
        return driverOp.getRightX();
    }

    public void telemetryUpdate() {
        telemetry.update();
    }

    public void dashboardTelemetryUpdate() {
        dashboardTelemetry.update();
    }
}