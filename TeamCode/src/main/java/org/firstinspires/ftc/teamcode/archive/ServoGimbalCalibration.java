package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mainModules.Presses;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


//this file was created for calculating the actual rom of the servos, it was calculated that the rom is about 245.64 degrees instead of 270 that rev said wtf?

@Disabled
@TeleOp(name = "ServoGimbalCalibration")
public class ServoGimbalCalibration extends LinearOpMode {
    private final String archiveStorage = "/sdcard/calibration/ServoGimbalCalibration/archive.txt";
    private final String currentConfig = "/sdcard/calibration/ServoGimbalCalibration/current.txt";

    //add library
    private static AprilTagLibrary getFeedingTheFutureTagLibrary() {
        return (new AprilTagLibrary.Builder())
                .addTag(100, "Blue Nexus Goal - Field Center - Facing Platform", 160.0, DistanceUnit.MM)
                .addTag(101, "Red Nexus Goal - Field Center - Facing Platform", 160.0, DistanceUnit.MM)
                .addTag(102, "Red Nexus Goal - Field Center - Facing Food Warehouse", 160.0, DistanceUnit.MM)
                .addTag(103, "Blue Nexus Goal - Field Center - Facing Food Warehouse", 160.0, DistanceUnit.MM)
                .addTag(104, "Blue Nexus Goal - Field Edge - Alliance Station", 160.0, DistanceUnit.MM)
                .addTag(105, "Blue Nexus Goal - Field Edge - Center Field", 160.0, DistanceUnit.MM)
                .addTag(106, "Red Nexus Goal - Field Edge - Center Field", 160.0, DistanceUnit.MM)
                .addTag(107, "Red Nexus Goal - Field Edge - Alliance Station", 160.0, DistanceUnit.MM)
                .build();
    }

    // Declare servo and AprilTag detection objects
    private Servo servoPort1;
    private Servo servoPort0;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private final ElapsedTime clock = new ElapsedTime();

    Presses pass = new Presses();

    private static final AprilTagLibrary feedingTheFutureTagLibrary = getFeedingTheFutureTagLibrary();


    @Override
    public void runOpMode() {

        double guessedROM = 245.64;
        double guessedROMRad = Math.toRadians(guessedROM);

        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(feedingTheFutureTagLibrary)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false) //save resources
                .setDrawTagOutline(false) //save resources
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);
        // Initialize the servos
        servoPort1 = hardwareMap.get(Servo.class, "Servo_Port_1_CH");
        servoPort0 = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
        AnalogInput potentiometer = hardwareMap.get(AnalogInput.class, "Analog_Port_0_CH");

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);


        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            servoPort1.setPosition(0.71);
            servoPort0.setPosition(0.932);
            double startTime = clock.milliseconds();
            while (clock.milliseconds() - startTime < 3000) {
                if (isStopRequested()){
                    break;
                }
            }

            AprilTagDetection detection = aprilTag.getDetections().get(0);
            double andgle1 = Math.atan(detection.ftcPose.z/detection.ftcPose.y);

           servoPort0.setPosition(0.932 + 2* andgle1/guessedROMRad);

            startTime = clock.milliseconds();
            while (clock.milliseconds() - startTime < 3000) {
                if (isStopRequested()){
                    break;
                }
            }

            detection = aprilTag.getDetections().get(0);
            double andgle2 = Math.atan(detection.ftcPose.z/detection.ftcPose.y);

            telemetry.addLine("finished");
            telemetry.addData("angle1", andgle1);
            telemetry.addData("angle2", andgle2);
            telemetry.addData("diff", andgle2+andgle1);
            telemetry.addData("realROM", guessedROM + (andgle2+andgle1)*guessedROM);
            telemetry.update();
            while (true){ //!pass.pressed(gamepad2.cross)
                if (isStopRequested()){
                    break;
                }
            }
        }
    }
}
