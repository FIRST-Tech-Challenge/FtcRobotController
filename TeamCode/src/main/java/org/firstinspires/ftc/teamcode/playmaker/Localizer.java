package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Localizer {

    private OpenGLMatrix cameraMatrix;
    private List<VuforiaTrackable> vuforiaTrackables = new ArrayList<VuforiaTrackable>();
    private VuforiaTransform lastVuforiaTransform;
    private Orientation lastIMUOrientation;
    private Orientation imuToWorld;

    // Vuforia Constants
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;


    /**
     * Return the robot's estimated location on the field.
     * X: the axis running from the audience view (negative) to the goals (positive)
     * Y: the axis running from red alliance (negative) to the blue alliance (positive)
     * Z: height of the robot off ground, though not relevant for this game.
     * @return the estimated Position
     */

    public Position estimatePosition() {
        if (lastVuforiaTransform != null) {
            VectorF translation = lastVuforiaTransform.transform.getTranslation();
            float x = translation.get(0);
            float y = translation.get(1);
            float z = translation.get(2);
            Position position = new Position(DistanceUnit.MM, x, y, z, System.currentTimeMillis());
            return position;
        }
        return null;
    }

    public Orientation estimateOrientation() {
        if (lastVuforiaTransform != null) {
            Orientation rotation = Orientation.getOrientation(lastVuforiaTransform.transform, EXTRINSIC, XYZ, DEGREES);
            return rotation;
        }
        return null;
    }



    /**
     * Sets the camera matrix for navigation.
     * @param hardware Robot Hardware being used
     * @param newCameraMatrix The new camera matrix to be applied
     */
    public void setCameraMatrix(RobotHardware hardware, OpenGLMatrix newCameraMatrix) {
        this.cameraMatrix = newCameraMatrix;
        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setPhoneInformation(this.cameraMatrix, hardware.vuforiaParameters.cameraDirection);
        }
    }

    /**
     * Sets the camera matrix for navigation using given offset and rotation.
     * @param hardware Robot Hardware being used
     * @param cameraOffset Offset of the camera relative to the robot's center (0,0,0)
     * @param cameraRotation Rotation of the camera relative to the robot's forward direction
     */
    public void setCameraMatrix(RobotHardware hardware, Position cameraOffset, Orientation cameraRotation) {
        this.cameraMatrix = OpenGLMatrix.translation((float) cameraOffset.x, (float) cameraOffset.y, (float) cameraOffset.z);
        this.cameraMatrix = this.cameraMatrix.multiplied(cameraRotation.getRotationMatrix());
        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setPhoneInformation(this.cameraMatrix, hardware.vuforiaParameters.cameraDirection);
        }
    }

    /**
     * Load the Ultimate Goal Trackables. Largely copied from ConceptVuforiaUltimteGoalNavigationWebcam
     */
    public void loadUltimateGoalTrackables(RobotHardware hardware) {
        VuforiaTrackables targetsUltimateGoal = hardware.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        vuforiaTrackables.addAll(targetsUltimateGoal);

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

        // Set the position of the perimeter targets with relation to origin (center of field)
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

        targetsUltimateGoal.activate();
    }

    public void updateLocationWithVuforia(RobotHardware hardware) {
        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            if (listener.isVisible()) {
                VectorF translation = listener.getVuforiaCameraFromTarget().getTranslation();
                hardware.telemetry.addData("Localizer Visible Target:", trackable.getName());
                hardware.telemetry.addData("Localizer Visible Target Rel Camera", String.format("%.1f, %.1f, %.1f", translation.get(0), translation.get(1), translation.get(2)));
                OpenGLMatrix robotTransform = listener.getUpdatedRobotLocation();
                if (robotTransform != null) {
                    lastVuforiaTransform = new VuforiaTransform(robotTransform);
                }
            }
        }
    }

    /**
     * This will attempt to find the differences of orientation between the field-centric system and
     * the IMU reference system.
     * @return Whether calibration was successful
     */

    public boolean attemptIMUToWorldCalibration() {

    }

    public void updateIMUOrientation(BNO055IMU imu) {
        Orientation imuOrientation = imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES);

        // TODO: Appply IMU -> World transformation
        this.lastIMUOrientation = imuOrientation;
    }


    public class VuforiaTransform {
        public long acquisitionTime;
        public OpenGLMatrix transform;

        public VuforiaTransform(OpenGLMatrix transform) {
            this.acquisitionTime = System.nanoTime();
            this.transform = transform;

        }
    }

    /**
     * Compute the XY distance between two Positions
     * @param a Position 1
     * @param b Position 2
     * @return XY Distance
     */
    public static double distance(Position a, Position b) {
        return Math.hypot(b.x - a.x, b.y - a.y);
    }

    /**
     *
     * @param start Starting angle
     * @param end End angle
     * @return An angle from -180 to 180, where positive angles indicate a rotation to the left and vice versa.
     */
    public static double angularDifference(double start, double end) {
        return (start - end + 180) % 360 - 180;
    }

    public static double headingWrapDegrees(double angle) {
        return (angle + 180) % 360 - 180;
    }

    public static double headingWrapRadians(double angle) {
        return (angle + Math.PI) % 2*Math.PI - Math.PI;
    }

    public static double atan2(Position a, Position b) {
        return Math.atan2(b.y-a.y,b.x-a.x);
    }

}
