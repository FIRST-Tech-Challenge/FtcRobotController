package org.firstinspires.ftc.teamcode.systems;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controls and updates the robot's odometry from the wheel encoders and IMU.
 * Because the system uses dead reckoning (no external encoders), the error of the
 * system will increase over time, particularly if there is a contact with another robot.
 */
public class OdometrySystem extends SubsystemBase {
    // FIXME: These values aren't correct
    Translation2d frontLeftLoc = new Translation2d(0, 0);
    Translation2d frontRightLoc = new Translation2d(0, 0);
    Translation2d backLeftLoc = new Translation2d(0, 0);
    Translation2d backRightLoc = new Translation2d(0, 0);
    
    Motor frontLeftMotor;
    Motor frontRightMotor;
    Motor backLeftMotor;
    Motor backRightMotor;
    
    MecanumDriveKinematics kinematics;
    MecanumDriveOdometry odometry;
    
    Pose2d pose;
    
    // FIXME: The control hub is slightly rotated, so these values need to be adjusted.
    //  The Z rotation should be increasing when the robot is turning left
    double xRot = 0, yRot = 0, zRot = 0;
    
    IMU imu;
    ElapsedTime elapsedTime;
    
    public OdometrySystem(final HardwareMap hardwareMap, Pose2d start, ElapsedTime elapsedTime) {
        this.elapsedTime = elapsedTime;
        
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftMotor = new Motor(hardwareMap, "front_left_drive");
        frontRightMotor = new Motor(hardwareMap, "front_right_drive");
        backLeftMotor = new Motor(hardwareMap, "rear_left_drive");
        backRightMotor = new Motor(hardwareMap, "rear_right_drive");

        Orientation hubRot = xyzOrientation(xRot, yRot, zRot);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRot);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        kinematics = new MecanumDriveKinematics(
            frontLeftLoc, 
            frontRightLoc, 
            backLeftLoc, 
            backRightLoc
        );
        
        odometry = new MecanumDriveOdometry(kinematics, getAngle(), start);
    }

    @Override
    public void periodic() {
        // FIXME: These need to be set to meters/sec
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
            frontLeftMotor.getRate(),
            frontRightMotor.getRate(),
            backLeftMotor.getRate(),
            backRightMotor.getRate()
        );
        
        pose = odometry.updateWithTime(elapsedTime.seconds(), getAngle(), wheelSpeeds);
    }
    
    private Rotation2d getAngle() {
       return Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
