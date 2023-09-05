package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "AprilTagTrack Iterative")
//@Disabled
public class AprilTagTrackingIterative extends OpMode {

    private static final boolean USE_WEBCAM = false;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    double loopTime;
    double priorTime = 0.0;
    static final double INITIAL_WHEEL_ANGLE = (Math.PI/2.0); // Wheel angle in radians
    TwoWheelDiffSwerveClass drive;

    // Potentiometer Class
    TwoFullRotationPotClass pots;

    double rotationAngle= 0;

    @Override
    public void init() {

        initAprilTag();
        drive = new TwoWheelDiffSwerveClass();
        drive.initDrive(hardwareMap);

        pots = new TwoFullRotationPotClass();
        pots.initPots(hardwareMap);
        pots.getAngleFromPots(false, 0); // find out where the wheels are pointed

        drive.initWheelAngles(pots.angle1, pots.angle2, INITIAL_WHEEL_ANGLE, INITIAL_WHEEL_ANGLE);  // set the wheels to desired angle

        // Send telemetry message to signify robot waiting;
        telemetry.addData("INITIAL POT 1 =", " %.05f, POT 2 = %.05f", pots.angle1, pots.angle2);
        pots.getAngleFromPots(false, 0); // find out where the wheels are pointed
        telemetry.addData("NEW POT 1 =", " %.05f, POT 2 = %.05f", pots.angle1, pots.angle2);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

        drive.setMotorsPower(0.8); // full speed  = 1.0. Backed off to prevent damage while developing
    }
    @Override
    public void loop() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            rotationAngle = 0;
            if (detection.metadata != null) {
                rotationAngle =  detection.ftcPose.bearing*Math.PI/180.0;
                drive.setMotorPositions(0,0,-(rotationAngle/20.0)); // rotates the robot
            }
        }   // end for() loop
        telemetry.addData("ROTATION ANGLE = ", rotationAngle);

        // Push telemetry to the Driver Station.
        telemetry.update();

        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }

        loopTime = getRuntime() - priorTime;
        priorTime = getRuntime();
        RobotLog.d("AprilTagITERATIVE = %.05f, loop = %.05f, rotationAngle = %.05f, robotangle = %.05f",priorTime,loopTime,rotationAngle,drive.robotAngle);

    }

}   // end class
