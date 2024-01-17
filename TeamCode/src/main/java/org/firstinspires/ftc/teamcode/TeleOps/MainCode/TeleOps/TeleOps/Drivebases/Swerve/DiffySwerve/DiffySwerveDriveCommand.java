package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Swerve.DiffySwerve;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

public class DiffySwerveDriveCommand extends CommandBase {

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final ModuleLeft leftModule;
    private final ModuleRight rightModule;

    private final GamepadEx driverGamepad;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;
    private SimpleTimer timer = new SimpleTimer();


    public DiffySwerveDriveCommand(ModuleLeft leftModule, ModuleRight rightModule, GamepadEx driverGamepad) {
        addRequirements(leftModule, rightModule);

        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5)
                // Add wheel locations
        );

        this.odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());

        this.leftModule = leftModule;
        this.rightModule = rightModule;

        this.driverGamepad = driverGamepad;

        this.xController = new PIDController(1.0, 0, 0);
        this.yController = new PIDController(1.0, 0, 0);
        this.rotController = new PIDController(1.0, 0, 0);
    }

    @Override
    public void initialize() {
        odometry.resetPosition(new Pose2d(), new Rotation2d());
    }

    @Override
    public void execute() {
        // Update odometry
        odometry.updateWithTime(
                timer.getElapsedTime(),  // Pass the current time
                new Rotation2d(driverGamepad.getLeftX()),  // Pass the gyro angle
                leftModule.getState(),
                rightModule.getState()
        );

        // Get joystick values
        double x = driverGamepad.getLeftX();
        double y = driverGamepad.getLeftY();
        double rot = driverGamepad.getRightX();

        // Calculate chassis speeds using PID controllers
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                xController.calculate(x, odometry.getPoseMeters().getTranslation().getX()),
                yController.calculate(y, odometry.getPoseMeters().getTranslation().getY()),
                rotController.calculate(rot, odometry.getPoseMeters().getRotation().getRadians())
        );

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Update swerve modules
        leftModule.setDesiredState(moduleStates[0]);
        rightModule.setDesiredState(moduleStates[1]);

        JoystickInfo joystickInfo = new JoystickInfo(driverGamepad);

        double leftStickAngle = joystickInfo.getLeftStickAngle();
        double leftStickMagnitude = joystickInfo.getLeftStickMagnitude();
        double leftTriggerValue = joystickInfo.getLeftTrigger();
        boolean isLeftStickButtonPressed = joystickInfo.isLeftStickButtonPressed();
    }

    @Override
    public void end(boolean interrupted) {
        leftModule.stop();
        rightModule.stop();
    }
}

class SimpleTimer {
    private long startTime;

    public SimpleTimer() {
        startTime = System.currentTimeMillis();
    }

    public double getElapsedTime() {
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }
}

class JoystickInfo {

    private GamepadEx gamepad;

    public JoystickInfo(GamepadEx gamepad) {
        this.gamepad = gamepad;
    }

    public double getLeftStickAngle() {
        double x = gamepad.getLeftX();
        double y = gamepad.getLeftY();
        return Math.toDegrees(Math.atan2(y, x));
    }

    public double getRightStickAngle() {
        double x = gamepad.getRightX();
        double y = gamepad.getRightY();
        return Math.toDegrees(Math.atan2(y, x));
    }

    public double getLeftStickMagnitude() {
        double x = gamepad.getLeftX();
        double y = gamepad.getLeftY();
        return Math.hypot(x, y);
    }

    public double getRightStickMagnitude() {
        double x = gamepad.getRightX();
        double y = gamepad.getRightY();
        return Math.hypot(x, y);
    }

    public double getLeftTrigger() {
        return gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    public double getRightTrigger() {
        return gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public boolean isLeftStickButtonPressed() {
        return gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
    }

    public boolean isRightStickButtonPressed() {
        return gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
    }
}
