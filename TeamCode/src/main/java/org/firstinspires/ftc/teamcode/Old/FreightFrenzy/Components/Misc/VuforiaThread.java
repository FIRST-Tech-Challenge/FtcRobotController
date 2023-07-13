/*
 * Vuforia Class. This is for detecting the
 * VuMarks. This class is for using the Webcams
 * It should display the target
 * seen, the location of the phone/robot
 * relative to the target, and the rotation/
 * angle of the phone in telemetry.
 *
 * @author  William
 * @version 1.0
 * @since   2021-September-24
 * @status: Fully working
 */

package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.OdometryChassis.vuforia_on;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.OdometryChassis;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;

public class VuforiaThread extends Thread {
    private static float xpos, ypos, angle;

    //It is used on line 193 for debugging purposes. This is why it isn't currently in use.
    private String trackable;

    private static boolean targetVisible = false;

    private VuforiaTrackables targets;
    private final List<VuforiaTrackable> allTrackables = new ArrayList<>();

    private OpenGLMatrix lastLocation = null;
    Lock location;

    private final float mmPerInch = 25.4f;

    public VuforiaThread(OpMode opMode, Lock locationMain) {

        // Vuforia License Key
        final String VUFORIA_KEY = "ATUOrmn/////AAABmVLVlWBtWUpnh9+EekIwR4lmMDXtnMrh/37lRyh+1m4oZJv1ANDvpS7D/Es9GNQ0wAkJ4YOHVWFjjsE5ptAFY2NRCAAwEY4VtvXEvSr3j/a0WR54dNfoCHRsnEaL5oQu25MoyOo7VrmhkE3xb2J9cNbsJzeqNaZWdQQpHkrgzEotos4i2tf/z+IMQxQ5nwH7Daiar93yoFv6FKeTh9MfI3bxVKR0nF+vrMzmNPC6YLk3yjqAKLqSgAvV0t07MBz9BjT2r58njS6qCo2U1H3sQXBlUcMdeKi4iclQaM+Oac+mZrzrhMvSEW7gC9mDhoL8l3zf2yMLPV9oGtnirNWn7ov/mupDtDecOUI4MPDNi9dt";

        // Initialize Variables

        final float halfField = 72 * mmPerInch;
        final float quarterField = 36 * mmPerInch;
        final float mmTargetHeight = 5.75f * mmPerInch;
         final float halfTile         = 12 * mmPerInch;
         final float oneAndHalfTile   = 36 * mmPerInch;

        WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "WebcamSide");


        // Show camera view on RC phone screen
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Set parameters
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        // Start Vuforia
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targets = vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

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

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        // Set Webcam Location
        final float CAMERA_FORWARD_DISPLACEMENT = -5.55f;//-1.625f
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.625f;//9.125f
        final float CAMERA_LEFT_DISPLACEMENT = -8.625f;//-8.9375f

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 180, -90, 0)); //YZX 0, 90, -90 //XYZ 0, -90, 0

        // Give Phone Location to Trackable Listeners
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Activate Vuforia Tracking
        targets.activate();
        location = locationMain;
    }

    public void run() {
        // Run until Thread is Interrupted
        while (!isInterrupted()) {
            if(vuforia_on){
                targetVisible = false;
                // Look for Trackable, Update Robot Location if Possible
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        // op.telemetry.addData("Visible Target ", trackable.getName());
                        targetVisible = true;

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

                // Return Location Data (Last Known Location)
                if (targetVisible) {
                    //setInVuforia(true);
                    VectorF translation = lastLocation.getTranslation();
//                op.telemetry.addData("Pos (in)", "{X, Y} = %.1f, %.1f",translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
//                op.telemetry.addData("Pos (in)", "{X, Y, Angle, getX, getY} = %.1f, %.1f, %.1f, %.1f, %.1f",translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, getCurrentAngle(), getXpos(), getYpos());
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                if (Math.sqrt(Math.pow(VuforiaWebcam.getVuforiaX(), 2) + Math.pow(VuforiaWebcam.getVuforiaY(), 2)) >= 24.5 && VuforiaWebcam.isTargetVisible() == true) {
//                    setXposition(translation.get(0) / mmPerInch);
//                    setYposition((translation.get(1) / mmPerInch));
//                    op.telemetry.addData("PosIf (in)", "{X, Y, getX, getY} = %.1f, %.1f, %.1f, %.1f",translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, getVuforiaX(), getVuforiaY());
                    xpos = translation.get(0) / mmPerInch;
                    ypos = translation.get(1) / mmPerInch;
                    angle = (rotation.thirdAngle + 180) % 360;

                    if (xpos > -15 && xpos < 15 && ypos > -60 && ypos < -38 && location.tryLock()) {
                        try {
                            location.lock();
                            OdometryChassis.setXpos(-xpos);
                            OdometryChassis.setYpos(ypos);
                            OdometryChassis.setAngle(angle);
                        }
                        finally{
                            location.unlock();
                        }

                    }
//                    op.telemetry.addData("PosIf (in)", "{X, Y, Angle, getX, getY} = %.1f, %.1f, %.1f, %.1f, %.1f",translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, getCurrentAngle(), getXpos(), getYpos());

//                    op.telemetry.addData("OVERWRITING...", null);
//                    op.telemetry.update();
//                }
                    try {
                        sleep(200);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }

                //setInVuforia(false);
                //op.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
//            else {
//                op.telemetry.addData("Visible Target", "none");
//            }
        }
    }

    public static double getVuforiaX() {
        return xpos;
    }

    public static double getVuforiaY() {
        return ypos;
    }

    public static double getVuforiaAngle() {
        return angle;
    }

    public static boolean isTargetVisible() {
        return targetVisible;
    }

    public double getVuforiaAngle2() {
        double vuforiaAngle = 90.0;
        return vuforiaAngle;
    }

    /*For Debugging
    public String getVuforiaTrackable() {
        return trackable;
    }
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}