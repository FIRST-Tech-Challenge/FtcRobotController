package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static org.firstinspires.ftc.teamcode.Measurement.*;

public class FieldTracker {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //

    public final boolean USING_WEBCAM;
    public final boolean USING_GRAPHICS;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private /*static*/ final boolean PHONE_IS_PORTRAIT;

    private static final String VUFORIA_KEY = "ARpGp0L/////AAABmY6ADA+dukfMs/X24JXU8YRheKdGFV7szbwDkeI7jTplRDGmGnMA+BeTijOEFY0pIFGENtdE8mpCFNSoDzG/nOdm93IHmNj/ZDz2FW91f8iv8loCXPGkQ7WncbiSLvPI4xgqFUDPqGhnoPfbCzdaD4anZ3lN62ViI6ltBRZpGoesIx1f3d06R0wDQtEZ6xuPl8Io9nWfElWlhTOY3yWnK2nXjnVw7ClPTduID/ODgFUyUMQ6G+xDhpDM5mLbq8r2macx7sRJzT500eoStVs55R4+Jm/VsifUCqW0LHpPm4g8u+2c5NRN+d3vJzOy0I8DWgpCfACFv59qriM9xDyHn4UemqcPWgLVyr3DU2p5dGN5";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
//    private static final float halfField = 72 * mmPerInch;
//    private static final float quadField  = 36 * mmPerInch;

    // adjust based on pictures being 36" from the side walls and wall to wall distance is 11' 9"
    private static final float halfField = 70.5f * mmPerInch;
    private static final float quadField  = 34.5f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    //private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 90;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    List<VuforiaTrackable> allTrackables;
    VuforiaTrackables targetsSkyStone;

    //Distances from camera/webcam to center of robot
    final static float CAMERA_FORWARD_DISPLACEMENT = 0;
    final static float CAMERA_VERTICAL_DISPLACEMENT = 0;
    final static float CAMERA_LEFT_DISPLACEMENT = 0;
    //TODO CHANGE THESE VALUES TO MATCH THE WEBCAM PLACEMENT
    final static float WEBCAM_FORWARD_DISPLACEMENT = 175; // 4.0f * mmPerInch;
    final static float WEBCAM_VERTICAL_DISPLACEMENT = 285; //4.625f * mmPerInch;
    final static float WEBCAM_LEFT_DISPLACEMENT = 0; //-6.5f * mmPerInch;

    public FieldTracker(HardwareMap m, Telemetry t, boolean usingWebcam, boolean usingGraphics) {
        USING_WEBCAM = usingWebcam;
        USING_GRAPHICS = usingGraphics;
        PHONE_IS_PORTRAIT = false; //was equal to usingWebcam

        hardwareMap = m;
        telemetry = t;

        setupTracker();
    }

    public void setupTracker() {

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */

        VuforiaLocalizer.Parameters parameters = (USING_GRAPHICS ?
                new VuforiaLocalizer.Parameters(
                        hardwareMap.appContext.getResources().getIdentifier(
                                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
                        )
                ) :
                new VuforiaLocalizer.Parameters()
        );

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (USING_WEBCAM) {
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        } else {
            parameters.cameraDirection = CAMERA_CHOICE;
        }

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.

        OpenGLMatrix robotFromCamera = ( USING_WEBCAM ?
                OpenGLMatrix.translation(WEBCAM_FORWARD_DISPLACEMENT, WEBCAM_LEFT_DISPLACEMENT, WEBCAM_VERTICAL_DISPLACEMENT) :
                OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT) )
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /*  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();

    }

    public TargetInfo getTargetInfo() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

                VectorF translation = lastLocation.getTranslation();
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, YZX, DEGREES);

                return new TargetInfo(rotation.thirdAngle, rotation.firstAngle, rotation.secondAngle,
                        convert(translation.get(0), MM_UNIT, CM_UNIT),
                        convert(translation.get(1), MM_UNIT, CM_UNIT),
                        convert(translation.get(2), MM_UNIT, CM_UNIT),
                        trackable.getName()
                );
            }
        }
        return null;
    }
}

class TargetInfo {
    double xRotation, yRotation, zRotation;
    double xPosition, yPosition, zPosition;
    String name;

    public TargetInfo(double xRot, double yRot, double zRot,
                      double xPos, double yPos, double zPos,
                      String name) {

        xRotation = xRot;
        yRotation = yRot;
        zRotation = zRot;

        xPosition = xPos;
        yPosition = yPos;
        zPosition = zPos;

        this.name = name;
    }

    public String toString() {
        return  "\nRotation\nx: " + xRotation + "\ny: " + yRotation + "\nz: " + zRotation +
                "\nTranslation\nx: " + xPosition + "\ny: " + yPosition + "\nz: " + zPosition +
                "\nName: " + name;
    }
}

class Position2D {
    double x, y;

    public Position2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getDistance(Position2D other) {
        return Math.sqrt((other.x - x) * (other.x - x) + (other.y - y) + (other.y - y));
    }
}

class Position3D {
    double x, y, z;

    public Position3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}