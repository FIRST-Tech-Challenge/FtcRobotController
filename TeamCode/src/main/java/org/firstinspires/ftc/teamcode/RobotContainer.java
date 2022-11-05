package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dragonswpilib.Command;
import org.firstinspires.ftc.teamcode.dragonswpilib.JoystickButton;

public class RobotContainer {

    private final Gamepad mGamepad1, mGamepad2;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;

    private final VuforiaCurrentGame mVuforiaPOWERPLAY;

    private final DriveSubsystem mDriveSubsystem;

    private final DriveCommand mDriveCommand;

    private final AutonomousCommandGroup mAutonomousCommandGroup;

    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        // Initialize Vuforia
        mVuforiaPOWERPLAY = new VuforiaCurrentGame();
        // Initialize using external web camera.
        mVuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                mHardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Activate here for camera preview.
        mVuforiaPOWERPLAY.activate();

        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry, mVuforiaPOWERPLAY);

        mDriveCommand = new DriveCommand(mTelemetry, mDriveSubsystem, mGamepad1);

        mAutonomousCommandGroup = new AutonomousCommandGroup(mTelemetry, mDriveSubsystem);

        configureButtonBindings();
        configureDefaultCommands();
    }

    @Override
    protected void finalize() {
        // Don't forget to deactivate Vuforia before the garbage collector removes the DriveSubsystem from memory
        mVuforiaPOWERPLAY.deactivate();
        mVuforiaPOWERPLAY.close();
    }

    private void configureButtonBindings() {
        //JoystickButton DPadUp = new JoystickButton(mGamepad1, JoystickButton.XboxControllerConstants.kDpadUp);
        //DPadUp.whenPressed(mGoToFrontAngleCommand);
    }

    private void configureDefaultCommands(){
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
    }
}
