package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
//@Config // EXAMPLE - use @Config to add public variables to dashboard for realtime updating
public class Odometry extends SubsystemBase {

    // stored robot position (static) used to keep robot pose between opmodes
    // value persists even if new odometry system is created.
    private static Pose2d StoredRobotPose = new Pose2d(0,0, new Rotation2d(0));

    // Local objects and variables here
    private double previousLeftPos;
    private double previousRightPos;
    private double previousFrontPos;
    private double fieldX = 0.0;
    private double fieldY = 0.0;
    private double fieldAngle = 0.0;

    // variables used for displaying paths on dashboard field widget
    // arrays hold x and y points of currently shown path(s)
    // arrays are set to null if no path to be shown
    private double[] currentTrajectoryXpoints;
    private double[] currentTrajectoryYpoints;

    /** Place code here to initialize subsystem */
    public Odometry() {

        // initialize field position from stored value (i.e. previous op-mode)
        setCurrentPos(StoredRobotPose);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        double leftPos;
        double rightPos;
        double frontPos;

        leftPos = RobotContainer.odometryPod.getLeftEncoderDistance();
        rightPos = RobotContainer.odometryPod.getRightEncoderDistance();
        frontPos = RobotContainer.odometryPod.getFrontEncoderDistance();

        double leftChangePos;
        double rightChangePos;
        double frontChangePos;

        leftChangePos = leftPos - previousLeftPos;
        rightChangePos = rightPos - previousRightPos;
        frontChangePos = frontPos - previousFrontPos;

        previousLeftPos = leftPos;
        previousRightPos = rightPos;
        previousFrontPos = frontPos;

        // creating the value of sin theta (aka the angle of the hipotinuse)
        double theta = Math.asin((rightChangePos - leftChangePos)/RobotContainer.odometryPod.LATERAL_DISTANCE);

        // equation that tells us how much the robot has moved forward
        double ForwardChange = (leftChangePos + rightChangePos) / 2.0 ;

        // equation that tells us how much the robot has moved laterally
        double LateralChange = (frontChangePos - RobotContainer.odometryPod.FORWARD_OFFSET * Math.sin(theta));// Lateral means left to right

        double IMUHeading = Math.toRadians(RobotContainer.gyro.getYawAngle());

        double fieldForwardChange = ForwardChange * Math.cos(IMUHeading) - LateralChange * Math.sin(IMUHeading);

        double fieldLateralChange = ForwardChange * Math.sin(IMUHeading) + LateralChange * Math.cos(IMUHeading);

        fieldX += fieldForwardChange;// += means is equal to and add fieldForwardChange to itself

        fieldY += fieldLateralChange;// += means is equal to and add fieldLateralChange to itself

        fieldAngle = IMUHeading;

        RobotContainer.ActiveOpMode.telemetry.addData("fieldX",fieldX);
        RobotContainer.ActiveOpMode.telemetry.addData("fieldY",fieldY);

        // update FTC dashboard with latest odometry info - in separate function below for clarity
        UpdateDashBoard();

        // save position to data store, in case op mode ends
        StoredRobotPose = new Pose2d(fieldX, fieldY, new Rotation2d(fieldAngle));
    }

    // place special subsystem methods here
    public Pose2d getCurrentPos() {
       return new Pose2d(fieldX,fieldY,new Rotation2d(fieldAngle));
    }

    public void setCurrentPos(Pose2d pos){
        fieldX = pos.getX();
        fieldY = pos.getY();
        fieldAngle = pos.getHeading();
        RobotContainer.gyro.setYawAngle(Math.toDegrees(fieldAngle));
    }

    public void resetCurrentPos(){
        setCurrentPos(new Pose2d(0,0,new Rotation2d(0)));
    }


    // Updates dashboard field widget with robot odometry info
    private void UpdateDashBoard()
    {
        // robot outline (note: values intentionally left in inches)
        // 0,0 is center of robot
        Vector2d p1 = new Vector2d(9, 0);
        Vector2d p2 = new Vector2d(-9, 9);
        Vector2d p3 = new Vector2d(-9, -9);
        Vector2d p4 = new Vector2d(0,0);

        // define robot field position offset vector
        Vector2d pos = new Vector2d(fieldX*39.3701, fieldY*39.3701);

        // rotate outline by angle of odometry and then add x,y position offset
        Vector2d p1rotated=(p1.rotateBy(Math.toDegrees(fieldAngle))).plus(pos);
        Vector2d p2rotated=(p2.rotateBy(Math.toDegrees(fieldAngle))).plus(pos);
        Vector2d p3rotated=(p3.rotateBy(Math.toDegrees(fieldAngle))).plus(pos);
        Vector2d p4rotated=(p4.rotateBy(Math.toDegrees(fieldAngle))).plus(pos);

        // create field telemetry packet
        TelemetryPacket field = new TelemetryPacket();
        field.fieldOverlay()
                .drawGrid(0, 0, 144, 144, 7, 7)
                .strokeLine(p1rotated.getX(), p1rotated.getY(), p2rotated.getX(), p2rotated.getY())
                .strokeLine(p2rotated.getX(), p2rotated.getY(), p3rotated.getX(), p3rotated.getY())
                .strokeLine(p3rotated.getX(), p3rotated.getY(), p1rotated.getX(), p1rotated.getY())
                .fillCircle(p1rotated.getX(),p1rotated.getY(), 2)
                .fillCircle(p4rotated.getX(),p4rotated.getY(), 1);
        //.fillText("Origin", 0, 0, "4px Arial", Math.toRadians(90), false)
        //.setRotation(Math.toRadians(90))
        //.strokeRect(x,y,width,height)
        //.drawImage("/dash/ftc.jpg", 24, 24, 18, 18, Math.toRadians(90), 24, 24, false);

        // do we have a trajectory to plot?
        if (currentTrajectoryXpoints!=null && currentTrajectoryYpoints!=null)
            field.fieldOverlay().strokePolyline(currentTrajectoryXpoints, currentTrajectoryYpoints);

        // update field
        RobotContainer.DashBoard.sendTelemetryPacket(field);

        // Show data on dashboard
        // double value1 = 1.0;
        // double value2 = 5.0;

        // Method #1
        // RobotContainer.DBTelemetry.addData("Value 1a", value1);
        // RobotContainer.DBTelemetry.addData("Value 2a", value2);
        // RobotContainer.DBTelemetry.update();

        // Method #2
        // TelemetryPacket data = new TelemetryPacket();
        // data.put("Value 1b", value1);
        // data.put("Value 2b", value2);
        // RobotContainer.DashBoard.sendTelemetryPacket(data);
    }

    // display provided trajectory on the dashboard field widget
    // set trajectory to null turn off trajectory
    public void DisplayTrajectory (Trajectory trajectory) {

        if (trajectory!=null)
        {
            // translate provided trajectory into separate x,y points array for use by dashboard field widget
            int length = trajectory.getStates().size();
            currentTrajectoryXpoints = new double[length];
            currentTrajectoryYpoints = new double[length];
            for (int index = 0; index < length; ++index) {
                currentTrajectoryXpoints[index] = 39.3701*trajectory.getStates().get(index).poseMeters.getX();
                currentTrajectoryYpoints[index] = 39.3701*trajectory.getStates().get(index).poseMeters.getY();
            }
        }
        else
        {
            // no trajectory to display - set arrays to null
            currentTrajectoryXpoints = null;
            currentTrajectoryYpoints = null;
        }
    }

}