package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class OdometrySubsystem extends SubsystemBase {

    // Local objects and variables here

    private double previousLeftPos;
    private double previousRightPos;
    private double previousFrontPos;
    private double fieldX = 0.0;
    private double fieldY = 0.0;
    private double fieldAngle = 0.0;

    /** Place code here to initialize subsystem */
    public OdometrySubsystem() {

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
}