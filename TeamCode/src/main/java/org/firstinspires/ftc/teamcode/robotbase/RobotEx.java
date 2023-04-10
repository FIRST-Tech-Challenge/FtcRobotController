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

    protected GamepadExEx driverOp;
    protected GamepadExEx toolOp;

    protected MecanumDrivePPV2 drive = null;
    protected MecanumDriveCommand driveCommand = null;

    public Camera camera;

    protected HeadingControllerSubsystem gyroFollow;
    protected HeadingControllerSubsystem cameraFollow;
    protected final Boolean initCamera;

    protected IMUSubsystem gyro;

    public RobotEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadExEx driverOp,
                   GamepadExEx toolOp) {
        this(hardwareMap, telemetry, driverOp, toolOp, OpModeType.TELEOP, false, false);
    }

    public RobotEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadExEx driverOp,
                   GamepadExEx toolOp, OpModeType type, Boolean initCamera, Boolean useCameraFollower
    ) {
        this.initCamera = initCamera;
        initCommon(hardwareMap, telemetry, type);
        if (type == OpModeType.TELEOP) {
            initTele(hardwareMap, telemetry, driverOp, toolOp, useCameraFollower);
            opModeType = OpModeType.TELEOP;
        } else {
            initAuto(hardwareMap, telemetry);
            opModeType = OpModeType.AUTO;
        }
    }

    public void initCommon(HardwareMap hardwareMap, Telemetry telemetry, OpModeType type) {
        /////////////////////////////////////// Telemetries //////////////////////////////////////
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        this.telemetry = telemetry;

        //////////////////////////////////////////// IMU ///////////////////////////////////////////
        gyro = new IMUSubsystem(hardwareMap, this.telemetry, dashboardTelemetry);
        CommandScheduler.getInstance().registerSubsystem(gyro);

        ////////////////////////////////////////// Camera //////////////////////////////////////////
        if (this.initCamera) {
            camera = new Camera(hardwareMap, dashboard, telemetry);
//                    () -> this.driverOp.getButton(GamepadKeys.Button.BACK));
        }
        drive = new MecanumDrivePPV2(hardwareMap, type);
    }

    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
//        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);

        ////////////////////////// Setup and Initialize Mechanisms Objects /////////////////////////
        initMechanismsAutonomous(hardwareMap);
    }

    public void initTele(HardwareMap hardwareMap, Telemetry telemetry, GamepadExEx driverOp,
                         GamepadExEx toolOp, Boolean useCameraFollower) {
        ///////////////////////////////////////// Gamepads /////////////////////////////////////////
        this.driverOp = driverOp;
        this.toolOp = toolOp;

        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
//        drive = new MecanumDriveSubsystem(hardwareMap, frontLeftInvert, frontRightInvert,
//                rearLeftInvert, rearRightInvert);
        driveCommand = new MecanumDriveCommand(drive, this::drivetrainForward,
                this::drivetrainStrafe, this::drivetrainTurn, gyro::getRawYaw,
                () -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        CommandScheduler.getInstance().registerSubsystem(drive);
        drive.setDefaultCommand(driveCommand);

        /////////////////////////////////////// Gyro Follower //////////////////////////////////////
        driverOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(gyro::resetYawValue, gyro));

        gyroFollow = new HeadingControllerSubsystem(gyro::getYaw,
                gyro::findClosestOrientationTarget);
        new Trigger(() -> driverOp.getRightY() >= 0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(180), gyroFollow));
        new Trigger(() -> driverOp.getRightY() <= -0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(0), gyroFollow));
        new Trigger(() -> driverOp.getRightX() >= 0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(-90), gyroFollow));
        new Trigger(() -> driverOp.getRightX() <= -0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(90), gyroFollow));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(0)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(180)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(90)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(-90)));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(gyroFollow::toggleState, gyroFollow));

//        telemetryEx.addMonitor("Left X: ", () -> toolOp.getLeftX());
//
//        telemetryEx.addMonitor("Left Y: ", () -> toolOp.getLeftY());
//        telemetryEx.addMonitor("Right X: ", () -> toolOp.getRightX());
//        telemetryEx.addMonitor("Right Y: ", () -> toolOp.getRightY());

        ////////////////////////////////////// Camera Follower /////////////////////////////////////
//        if (initCamera && useCameraFollower) {
//            cameraFollow = new HeadingControllerSubsystem(camera::getObject_x);
//            driverOp.getGamepadButton(GamepadKeys.Button.START)
//                    .whenPressed(new InstantCommand(cameraFollow::toggleState, cameraFollow));
//        }

        ////////////////////////// Setup and Initialize Mechanisms Objects /////////////////////////
        initMechanismsTeleOp(hardwareMap);
    }

    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        // should be overridden by child class
    }

    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        // should be overridden by child class
    }

    public double drivetrainStrafe() {
        return driverOp.getLeftX();
    }

    public double drivetrainForward() {
        return driverOp.getLeftY();
    }

    public double drivetrainTurn() {
        if (gyroFollow.isEnabled())
            return -gyroFollow.calculateTurn();
//        if (initCamera && cameraFollow.isEnabled())
//            return cameraFollow.calculateTurn();

        return driverOp.getRightX();
    }

    public void telemetryUpdate() {
        telemetry.update();
    }

    public void dashboardTelemetryUpdate() {
        dashboardTelemetry.update();
    }
}