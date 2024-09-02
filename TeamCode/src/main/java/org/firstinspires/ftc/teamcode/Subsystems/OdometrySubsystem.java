package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class OdometrySubsystem extends SubsystemBase {

    // Local objects and variables here

    double previousLeftPos;
    double previousRightPos;
    double previousFrontPos;
    double fieldX = 0.0;
    double fieldY = 0.0;

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

        double IMUHeading = -Math.toRadians(RobotContainer.gyro.getYawAngle());

        double fieldForwardChange = ForwardChange * Math.cos(IMUHeading) - LateralChange * Math.sin(IMUHeading);

        double fieldLateralChange = ForwardChange * Math.sin(IMUHeading) + LateralChange * Math.cos(IMUHeading);

        fieldX += fieldForwardChange;// += means is equal to and add fieldForwardChange to itself

        fieldY += fieldLateralChange;// += means is equal to and add fieldLateralChange to itself

        RobotContainer.ActiveOpMode.telemetry.addData("fieldX",fieldX);
        RobotContainer.ActiveOpMode.telemetry.addData("fieldY",fieldY);

        RobotContainer.ActiveOpMode.telemetry.update();

    }

    // place special subsystem methods here

}