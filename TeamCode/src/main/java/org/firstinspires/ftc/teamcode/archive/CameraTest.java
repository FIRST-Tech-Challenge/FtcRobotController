
package archive;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import com.qualcomm.robotcore.hardware.AccelerationSensor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Disabled
@TeleOp(name="Camera_test")
public class CameraTest extends LinearOpMode{
    //Kõik staatilised väärtused ja nende tüübid nt int, float, imu

    private Servo gimbalPitch = null;
    private Servo gimbalYaw = null;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    IMU imu;
    AccelerationSensor accelerationSensor;
    @Override public void runOpMode(){
        //kõik muutujad ja inisisaliseerimine

        double currentPitch = 0.5;
        double currentYaw = 0.5;
        double sameXAprilTag=0;

        // Initialize the Apriltag Detection process
        initAprilTag();
        
        gimbalPitch = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
        gimbalPitch.setPosition(currentPitch);
        gimbalYaw = hardwareMap.get(Servo.class, "Servo_Port_1_CH");
        gimbalYaw.setPosition(currentYaw);

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            boolean targetFound = false;
            desiredTag  = null;
            telemetry.addData("apriltags",aprilTag.getDetections());


            for (AprilTagDetection detection : aprilTag.getDetections())  {
                if (detection!=null){
                 Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            
                 // Original source data
                 double poseX = detection.rawPose.x;
                 double poseY = detection.rawPose.y;
                 double poseZ = detection.rawPose.z;
                
                 double poseAX = rot.firstAngle;
                 double poseAY = rot.secondAngle;
                 double poseAZ = rot.thirdAngle;
                 telemetry.addData("xTag", poseX);
                 telemetry.addData("zTag", poseZ);
                 telemetry.addData("yTag", poseY);
                 telemetry.addData("compensation", -Math.toDegrees(Math.asin(poseX/poseZ))/270);
                 telemetry.addData("compensation", -Math.toDegrees(Math.asin(poseY/poseZ))/270);
                 telemetry.update();
                 if (true){
                     double yawAngleOffset=Math.toDegrees(Math.asin(poseX/poseZ));
                     currentYaw+=yawAngleOffset/27000;
                     
                     double pitchAngleOffset=Math.toDegrees(Math.asin(poseY/poseZ));
                     currentPitch-=pitchAngleOffset/27000;
                 }
                sameXAprilTag=poseX;
                 }
            }
            
            telemetry.update();

            if (gamepad1.dpad_up) { 
                //currentPitch += 0.001;
            }
            if (gamepad1.dpad_down) {
                currentPitch -= 0.001;
            }
            if (gamepad1.dpad_left) {
                currentYaw -= 0.001;
            }
            if (gamepad1.dpad_right) {
                currentYaw += 0.001;
            }
            gimbalPitch.setPosition(currentPitch);
            gimbalYaw.setPosition(currentYaw);
            sleep(10);
        }
    }



    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()){
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
