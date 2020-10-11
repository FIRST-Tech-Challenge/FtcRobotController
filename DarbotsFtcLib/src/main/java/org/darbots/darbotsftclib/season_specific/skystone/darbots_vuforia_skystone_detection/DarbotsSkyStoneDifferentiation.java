package org.darbots.darbotsftclib.season_specific.skystone.darbots_vuforia_skystone_detection;

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

public class DarbotsSkyStoneDifferentiation implements RobotNonBlockingDevice {

    private RobotPose3D m_CameraPos; //Distance in CM, Rotation in Darbots Transformation
    private RobotCamera m_Camera;
    private VuforiaTrackables m_TargetsSkyStone;
    private List<VuforiaTrackable> m_AllTrackables;

    public DarbotsSkyStoneDifferentiation(RobotPose3D CameraPosition, RobotCamera Camera){
        this.m_Camera = Camera;
        this.m_CameraPos = CameraPosition;
        m_AllTrackables = new ArrayList<VuforiaTrackable>();
        this.__setupVuforia();
    }

    public DarbotsSkyStoneDifferentiation(DarbotsSkyStoneDifferentiation oldNav){
        this.m_CameraPos = oldNav.m_CameraPos;
        this.m_Camera = oldNav.m_Camera;
        this.m_TargetsSkyStone = oldNav.m_TargetsSkyStone;
        this.m_AllTrackables = oldNav.m_AllTrackables;
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

    protected void __setupVuforia(){
        VuforiaLocalizer vuforia = this.m_Camera.getVuforia();
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        m_TargetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone_Darbots");

        VuforiaTrackable DarbotsStoneTarget = m_TargetsSkyStone.get(0);
        DarbotsStoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> m_AllTrackables = new ArrayList<VuforiaTrackable>();
        m_AllTrackables.addAll(m_TargetsSkyStone);

        __setupCamera();
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
        return;
    }

    @Override
    public void waitUntilFinish() {
        return;
    }


    public void setActivated(boolean enabled){
        if(enabled){
            this.m_TargetsSkyStone.activate();
        }else{
            this.m_TargetsSkyStone.deactivate();
        }
    }
}
