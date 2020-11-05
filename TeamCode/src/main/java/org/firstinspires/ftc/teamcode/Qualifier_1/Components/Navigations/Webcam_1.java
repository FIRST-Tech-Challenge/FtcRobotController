//package org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
//import java.util.ArrayList;
//import java.util.List;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//
//@TeleOp(name="Webcam_1")
//public class Webcam_1 extends LinearOpMode {
//
//    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
//    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//    private static final boolean PHONE_IS_PORTRAIT = false  ;
//
//    private static final String VUFORIA_KEY =
//            "ATUOrmn/////AAABmVLVlWBtWUpnh9+EekIwR4lmMDXtnMrh/37lRyh+1m4oZJv1ANDvpS7D/Es9GNQ0wAkJ4YOHVWFjjsE5ptAFY2NRCAAwEY4VtvXEvSr3j/a0WR54dNfoCHRsnEaL5oQu25MoyOo7VrmhkE3xb2J9cNbsJzeqNaZWdQQpHkrgzEotos4i2tf/z+IMQxQ5nwH7Daiar93yoFv6FKeTh9MfI3bxVKR0nF+vrMzmNPC6YLk3yjqAKLqSgAvV0t07MBz9BjT2r58njS6qCo2U1H3sQXBlUcMdeKi4iclQaM+Oac+mZrzrhMvSEW7gC9mDhoL8l3zf2yMLPV9oGtnirNWn7ov/mupDtDecOUI4MPDNi9dt";
//
//    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
//    // We will define some constants and conversions here
//    private static final float mmPerInch        = 25.4f;
//    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
//
//    // Constants for perimeter targets
//    private static final float halfField = 72 * mmPerInch;
//    private static final float quadField  = 36 * mmPerInch;
//
//    // Class Members
//    private OpenGLMatrix lastLocation = null;
//    private VuforiaLocalizer vuforia = null;
//
//    /**
//     * This is the webcam we are to use. As with other hardware devices such as motors and
//     * servos, this device is identified using the robot configuration tool in the FTC application.
//     */
//    WebcamName webcamName = null;
//
//    private boolean targetVisible = false;
//    private float phoneXRotate    = 0;
//    private float phoneYRotate    = 0;
//    private float phoneZRotate    = 0;
//
//    @Override public void runOpMode() {
//        /*
//         * Retrieve the camera we are to use.
//         */
//        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
//         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
//         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//
//        /**
//         * We also indicate which camera on the RC we wish to use.
//         */
//        parameters.cameraName = webcamName;
//
//        // Make sure extended tracking is disabled for this example.
//        parameters.useExtendedTracking = false;
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Load the data sets for the trackable objects. These particular data
//        // sets are stored in the 'assets' part of our application.
//        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
//        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
//        blueTowerGoalTarget.setName("Blue Tower Goal Target");
//        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
//        redTowerGoalTarget.setName("Red Tower Goal Target");
//        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
//        redAllianceTarget.setName("Red Alliance Target");
//        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
//        blueAllianceTarget.setName("Blue Alliance Target");
//        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
//        frontWallTarget.setName("Front Wall Target");
//
//        // For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(targetsUltimateGoal);
//
//
//        //Set the position of the perimeter targets with relation to origin (center of field)
//        redAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        blueAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//        frontWallTarget.setLocation(OpenGLMatrix
//                .translation(-halfField, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//
//        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
//        blueTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//        redTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        // Info:  The coordinate frame for the robot looks the same as the field.
//        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
//        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
//        //
//        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
//        // pointing to the LEFT side of the Robot.
//        // The two examples below assume that the camera is facing forward out the front of the robot.
//
//        // We need to rotate the camera around it's long axis to bring the correct camera forward.
//        if (CAMERA_CHOICE == BACK) {
//            phoneYRotate = -90;
//        } else {
//            phoneYRotate = 90;
//        }
//
//        // Rotate the phone vertical about the X axis if it's in portrait mode
//        if (PHONE_IS_PORTRAIT) {
//            phoneXRotate = 90 ;
//        }
//
//        // Next, translate the camera lens to where it is on the robot.
//        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
//        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
//        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
//        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
//
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//
//        /**  Let all the trackable listeners know where the phone is.  */
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//        }
//
//        targetsUltimateGoal.activate();
//        while (!isStopRequested()) {
//
//            // check all the trackable targets to see which one (if any) is visible.
//            targetVisible = false;
//            for (VuforiaTrackable trackable : allTrackables) {
//                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
//                    targetVisible = true;
//
//                    // getUpdatedRobotLocation() will return null if no new information is available since
//                    // the last time that call was made, or if the trackable is not currently visible.
//                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
//                    if (robotLocationTransform != null) {
//                        lastLocation = robotLocationTransform;
//                    }
//                    break;
//                }
//            }
//
//            // Provide feedback as to where the robot is located (if we know).
//            if (targetVisible) {
//                // express position (translation) of robot in inches.
//                VectorF translation = lastLocation.getTranslation();
//                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                // express the rotation of the robot in degrees.
//                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//            }
//            else {
//                telemetry.addData("Visible Target", "none");
//            }
//            telemetry.update();
//        }
//
//        // Disable Tracking when we are done;
//        targetsUltimateGoal.deactivate();
//    }
//}
