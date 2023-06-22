package org.firstinspires.ftc.teamcode.opModes.team1.auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.FEET_PER_METER;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.LEFT_TAG;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.MIDDLE_TAG;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.RIGHT_TAG;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.cx;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.cy;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.fx;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.fy;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.tagsize;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.components.HDriveWrapper;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class GenericAuto {
    Telemetry telemetry;

    public void run(TeamColour teamColour, AutonomousLinearModeBase autonomousModeBase) {
        telemetry = TelemetryContainer.getTelemetry();
        IMU imu = HardwareMapContainer.getMap().get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // Get the third motor as a spinner motor
        HDriveWrapper drive = new HDriveWrapper(new HDrive(
                HardwareMapContainer.motor0,
                HardwareMapContainer.motor1,
                HardwareMapContainer.motor3,
                0,
                Math.PI,
                Math.PI/2
        ), imu);
    }
}
