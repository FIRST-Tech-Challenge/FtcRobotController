package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

@TeleOp(name="testPlatfromTeleopWithCam", group="Zippo")
//@Disabled

public class testPlatformTeleopWithCam extends OpMode{

    testPlatformHardware robot  = new testPlatformHardware();

    // Create variables for motor power
    private double flPower = 0; //left wheel
    private double frPower = 0; //right wheel
    private double blPower = 0; //front power
    private double brPower = 0; //back power
    private double liftPower = 0; // lift power

    private boolean scalePower = false;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY = vuforia_key.key1;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;



    // Load the data sets for the trackable objects. These particular data
    // sets are stored in the 'assets' part of our application.


    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    @Override
    public void init() {
        robot.init(hardwareMap);
        double time =  System.currentTimeMillis();

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");

        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);

        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);

        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);

        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);

        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        redTowerGoalTarget.setName("Red Tower Goal Target");
        redAllianceTarget.setName("Red Alliance Target");
        blueAllianceTarget.setName("Blue Alliance Target");
        frontWallTarget.setName("Front Wall Target");

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.clear();
        allTrackables.addAll(targetsUltimateGoal);
        targetsUltimateGoal.activate();

    }

    @Override
    public void loop() {


        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */
        setPowers();


        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();

    }

//--------------------------------- FUNCTIONS ----------------------------------------------------

    public double scalePower(double power1)
    {
        double power2;
        if(power1 > 0){
            power2 = Math.pow(Math.abs(power1), 0.6);
        } else if(power1 < 0){
            power2 = -Math.pow(Math.abs(power1), 0.6);
        } else{
            power2 = 0;
        }

        return power2;
    }
    public void setPowers() {
        if ((gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1)||(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1))
        {
            if(scalePower == true) {
                flPower = gamepad1.left_stick_y * 0.5;
                brPower = gamepad1.left_stick_y * 0.5;
                frPower = gamepad1.left_stick_y * 0.5;
                blPower = gamepad1.left_stick_y * 0.5;
            }
            else{
                flPower = gamepad1.left_stick_y * 2;
                brPower = gamepad1.left_stick_y * 2;
                frPower = gamepad1.left_stick_y * 2;
                blPower = gamepad1.left_stick_y * 2;
            }
        }
        else if ((gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_x > .2) || (gamepad2.left_stick_x <-.02 || gamepad2.left_stick_x > .2))
        {
            if(scalePower == true) {
                frPower = gamepad1.left_stick_x * 0.5;
                blPower = gamepad1.left_stick_x * 0.5;
                flPower = -gamepad1.left_stick_x * 0.5;
                brPower = -gamepad1.left_stick_x * 0.5;
            }
            else{
                frPower = gamepad1.left_stick_x * 2;
                blPower = gamepad1.left_stick_x * 2;
                flPower = -gamepad1.left_stick_x * 2;
                brPower = -gamepad1.left_stick_x * 2;
            }

        }
        else
        {
            flPower = 0;
            brPower = 0;
            frPower = 0;
            blPower = 0;
        }

        if (gamepad1.right_stick_x < -0.1 || gamepad2.right_stick_x < -0.1) {
            flPower = 0.5;
            brPower = -0.5;
            frPower = -0.5;
            blPower = 0.5;
        } else if (gamepad1.right_stick_x > 0.1 || gamepad2.right_stick_x> 0.1) {
            flPower = -0.5;
            brPower = 0.5;
            frPower = 0.5;
            blPower = -0.5;
        }


        if(frPower > 1)
        {
            frPower = 1;
        }
        else if (frPower < -1)
        {
            frPower = -1;
        }

        if(flPower > 1)
        {
            flPower = 1;
        }
        else if (flPower < -1)
        {
            flPower = -1;
        }

        if(blPower > 1)
        {
            blPower = 1;
        }
        else if (blPower < -1)
        {
            blPower = -1;
        }
        if(brPower > 1)
        {
            brPower = 1;
        }
        else if (brPower < -1)
        {
            brPower = -1;
        }
//
        if (gamepad1.right_trigger > 0)
        {
            scalePower = true;
        }
        else
        {
            scalePower = false;
        }





//      SCALING POWERS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        if(scalePower)
        {
            flPower *= .7;
            brPower *= .7;
            frPower *= .7;
            blPower *= .7;
        }

//      SETTING POWERS AND POSITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        robot.motorFrontLeft.setPower(flPower);
        robot.motorBackRight.setPower(brPower);
        robot.motorFrontRight.setPower(frPower);
        robot.motorBackLeft.setPower(blPower);
    }
}