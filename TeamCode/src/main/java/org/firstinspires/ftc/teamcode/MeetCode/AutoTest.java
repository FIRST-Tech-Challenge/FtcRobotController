//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.TestHardware;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import java.util.List;
//
//
//@Autonomous(name = "EncoderTestForAutoRightBlue")
//
//public class EncoderTest extends LinearOpMode {
//    Hardware robot = new Hardware();
//
//    ConceptAprilTagEasy tag = new ConceptAprilTagEasy();
//    private int tagID;
//    private int numberOfAprilTags;
//    private double xDistance;
//    private double yDistance;
//    private double zDistance;
//    private List<AprilTagDetection> aprilTags;
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//    private AprilTagProcessor aprilTag;
//
//    private List<Recognition> currentRecognitions;
//    /**
//     * The variable to store our instance of the vision portal.
//     */
//    private VisionPortal visionPortal;
//    private TfodProcessor tfod;
//
//    private String LABEL = "";
//    private double x;
//    private double y;
//    /*
//     * Specify the source for the Tensor Flow Model.
//     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
//     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
//     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
//     * Here we assume it's an Asset.    Also see method initTfod() below .
//     */
//
////    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
////    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
////    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
////    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
////            (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//        waitForStart();
//
//        //add in init robot claw wrist as turned up
//        robot.turnOnEncoders();
//        robot.encoderDrive(29); //24
////        robot.currentEncoderPosition();
//        sleep(1000);
//
////        robot.turnOffEncoders();
////        //Move forward till the sensor sees blue
////        while (robot.colorSensor.blue() < (robot.colorSensor.red() + robot.colorSensor.green()))
////            robot.setPowerOfAllMotorsTo(0.1);
////        robot.setPowerOfAllMotorsTo(0);
////        robot.turnOnEncoders();
////        sleep(250);
//        //drop pixel
////        robot.encoderDrive(-3);
//        //drop
//        robot.dropper.setPosition(1);
//        //ig u could move it back
//        sleep(1000);
//        robot.dropper.setPosition(0);
//        robot.encoderDrive(-4);
//        sleep(250);
//        //turn or strafe?
//        robot.encoderStrafeLeft(84);
//        robot.encoderTurnLeft(21); //12.64
////        robot.turnOffEncoders();
//        sleep(250);
////        robot.encoderDrive(30);
////        //Sees AprilTag and try to square up
//        while (opModeIsActive()) {
//            robot.squareUp();
//            if (robot.withinDistanceRange())
//                break;
//        }
//        sleep(500);
//        if (findID(2)) {
//            //go to the coordinates for apriltag: X (Right), Y (Forward), Z (Up) dist.
//            robot.turnOffEncoders();
//            while (xDistance > 0.25 && xDistance < -0.25 ) {
//                if (xDistance > 0.25)
//                    //strafe right
//                    robot.strafeRight(0.1);
//                if (xDistance < -0.25)
//                    // strafe left
//                    robot.strafeLeft(0.1);
//
//            }
//            robot.setPowerOfAllMotorsTo(0);
//            while (yDistance > 12)
//                robot.setPowerOfAllMotorsTo(0.1);
//            robot.setPowerOfAllMotorsTo(0);
////            //sent out cascade and drop the pixel
////            robot.turnOnEncoders();
////            robot.turnOnEncodersCascade();
////            robot.cascadeDrive(8);
////            //open
////            robot.claw.setPosition(0.6);
////            //closed
////            robot.claw.setPosition(0);
////            //bring cascade back
////            robot.cascadeDrive(-8);
//////            //turn wrist up
////            robot.wrist.setPosition(1);
//////            //park: strafe left
////        robot.encoderStrafeLeft(24);
////        robot.encoderDrive(10);
//        }
//    }
//    private void telemetryAprilTag () {
//
////        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        aprilTags = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", aprilTags.size());
//        numberOfAprilTags = aprilTags.size();
//
//        // Step through the list of detections and display info for each one.
////        for (AprilTagDetection detection : aprilTags)
//        for (int i = 0; i < aprilTags.size(); i++) {
//            AprilTagDetection detection = aprilTags.get(i);
//            if (detection.metadata != null) {
////                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                tagID = detection.id;
////                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                xDistance = detection.ftcPose.x;
//                yDistance = detection.ftcPose.y;
//                zDistance = detection.ftcPose.z;
////                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//////                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
////                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                tagID = detection.id;
////                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//                xDistance = detection.center.x;
//                yDistance = detection.center.y;
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
////        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
////        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
////        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//    }   // end method telemetryAprilTag()
//    private boolean findID ( int num){
//        for (int i = 0; i < aprilTags.size(); i++) {
//            if (num == aprilTags.get(i).id)
//                return true;
//        }
//        return false;
//    }
//
//
//}
