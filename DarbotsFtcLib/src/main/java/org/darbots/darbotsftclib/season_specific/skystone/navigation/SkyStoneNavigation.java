package org.darbots.darbotsftclib.season_specific.skystone.navigation;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose3D;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
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

public class SkyStoneNavigation implements RobotNonBlockingDevice {
    private static final float MMPERINCH        = 25.4f;
    private static final float MMTARGETHEIGHT   = (6) * MMPERINCH;
    // Constant for Stone Target
    private static final float stoneZ = 2.00f * MMPERINCH;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * MMPERINCH;
    private static final float bridgeY = 23 * MMPERINCH;
    private static final float bridgeX = 5.18f * MMPERINCH;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * MMPERINCH;
    private static final float quadField  = 36 * MMPERINCH;

    private RobotPose3D m_CameraPos; //Distance in CM, Rotation in Darbots Transformation
    private RobotCamera m_Camera;
    private VuforiaTrackables m_TargetsSkyStone;
    private List<VuforiaTrackable> m_AllTrackables;
    private ElapsedTime m_LastTime;
    private RobotPose3D m_LastPosition;
    private boolean m_LastUpdateGotLocation;

    public SkyStoneNavigation(RobotPose3D CameraPosition, RobotCamera Camera){
        this.m_LastUpdateGotLocation = false;
        this.m_Camera = Camera;
        this.m_CameraPos = CameraPosition;
        m_AllTrackables = new ArrayList<VuforiaTrackable>();
        this.m_LastPosition = null;
        this.m_LastTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.__setupVuforia();
    }

    public SkyStoneNavigation(SkyStoneNavigation oldNav){
        this.m_LastUpdateGotLocation = false;
        this.m_CameraPos = oldNav.m_CameraPos;
        this.m_Camera = oldNav.m_Camera;
        this.m_TargetsSkyStone = oldNav.m_TargetsSkyStone;
        this.m_AllTrackables = oldNav.m_AllTrackables;
        this.m_LastTime = oldNav.m_LastTime;
        this.m_LastPosition = oldNav.m_LastPosition;
        this.__setupVuforia();
    }

    public RobotCamera getCamera(){
        return this.m_Camera;
    }
    public RobotPose3D getCameraPosition(){
        return this.m_CameraPos;
    }
    public void setCameraPosition(RobotPose3D CameraPosition){
        this.m_CameraPos = CameraPosition;
        this.__setupCamera();
    }

    public RobotPose3D getDarbotsFieldPosition(){
        return this.__getFTCFieldPosition();
    }

    protected void __setupVuforia(){
        VuforiaLocalizer vuforia = this.m_Camera.getVuforia();
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        m_TargetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        //m_TargetsSkyStone.remove(0);

        VuforiaTrackable stoneTarget = m_TargetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = m_TargetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = m_TargetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = m_TargetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = m_TargetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = m_TargetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = m_TargetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = m_TargetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = m_TargetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = m_TargetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = m_TargetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = m_TargetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = m_TargetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> m_AllTrackables = new ArrayList<VuforiaTrackable>();
        m_AllTrackables.addAll(m_TargetsSkyStone);

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

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        /*
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        */

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
                .translation(quadField, -halfField, MMTARGETHEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, MMTARGETHEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, MMTARGETHEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, MMTARGETHEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, MMTARGETHEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, MMTARGETHEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, MMTARGETHEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, MMTARGETHEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        __setupCamera();

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
    }
    protected void __setupCamera(){
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        m_AllTrackables.clear();
        m_AllTrackables.addAll(m_TargetsSkyStone);
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

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        RobotPose3D FTCRobotPos = m_CameraPos;
        float CAMERA_X_DISPLACEMENT  = (float) (FTCRobotPos.X * 10.0);
        float CAMERA_Y_DISPLACEMENT = (float) (FTCRobotPos.Y * 10.0);
        float CAMERA_Z_DISPLACEMENT = (float) (FTCRobotPos.Z * 10.0);
        float CAMERA_X_ROTATION = (float) (FTCRobotPos.getRotationX());
        float CAMERA_Y_ROTATION = (float) (FTCRobotPos.getRotationY());
        float CAMERA_Z_ROTATION = (float) (FTCRobotPos.getRotationZ());

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_X_DISPLACEMENT, CAMERA_Y_DISPLACEMENT, CAMERA_Z_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, CAMERA_Y_ROTATION, CAMERA_Z_ROTATION, CAMERA_X_ROTATION));

        VuforiaLocalizer.CameraDirection cameraDirection = this.m_Camera instanceof RobotOnPhoneCamera ? ((RobotOnPhoneCamera) this.m_Camera).getVuforiaCameraDirection() : VuforiaLocalizer.CameraDirection.BACK;
        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : m_AllTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, cameraDirection);
        }
    }
    protected RobotPose3D __getFTCFieldPosition(){
        boolean targetVisible = false;
        OpenGLMatrix lastLocation = null;
        for (VuforiaTrackable trackable : m_AllTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() && (!trackable.getName().equals("Stone Target"))) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getFtcFieldFromRobot();

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

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return new RobotPose3D(translation.get(0) / 10,translation.get(1) / 10,translation.get(2) / 10, rotation.firstAngle,rotation.secondAngle,rotation.thirdAngle);
        }
        else {
            return null;
        }
    }

    protected RobotPose3D __getFTCRobotAxisStonePosition(){
        VuforiaTrackableDefaultListener trackable = (VuforiaTrackableDefaultListener) m_AllTrackables.get(0).getListener();
        if(trackable.isVisible()){
            OpenGLMatrix stonePosition = this.getCamera() instanceof RobotOnPhoneCamera ? trackable.getPosePhone() : trackable.getFtcCameraFromTarget();
            VectorF translation = stonePosition.getTranslation();
            Orientation rotation = Orientation.getOrientation(stonePosition,EXTRINSIC,XYZ,DEGREES);
            return new RobotPose3D(translation.get(0) / 10, translation.get(1) / 10, translation.get(2) / 10, rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        return null;
    }
    public RobotPose3D getDarbotsRobotAxisStonePosition(){
        RobotPose3D FTCRobotAxis = this.__getFTCRobotAxisStonePosition();
        if(FTCRobotAxis != null){
            return FTCRobotAxis;
        }
        return null;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void updateStatus() {
        RobotPose3D newestPos = this.getDarbotsFieldPosition();
        if(newestPos != null){
            this.m_LastTime.reset();
            this.m_LastPosition = newestPos;
            this.m_LastUpdateGotLocation = true;
        }else{
            this.m_LastUpdateGotLocation = false;
        }
    }

    @Override
    public void waitUntilFinish() {
        return;
    }

    public RobotPose3D getLastPosition(){
        return this.m_LastPosition;
    }

    public double getSecondsSinceLastPosition(){
        return this.m_LastTime.seconds();
    }

    public void clearLastPosition(){
        this.m_LastPosition = null;
        this.m_LastTime.reset();
        this.m_LastUpdateGotLocation = false;
    }

    public void setActivated(boolean enabled){
        if(enabled){
            this.m_TargetsSkyStone.activate();
        }else{
            this.m_TargetsSkyStone.deactivate();
        }
    }
}
