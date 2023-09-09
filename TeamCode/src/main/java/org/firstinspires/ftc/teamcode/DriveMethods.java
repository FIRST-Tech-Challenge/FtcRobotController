//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Translation2d;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import static org.firstinspires.ftc.teamcode.Variables.*;
//
//public class DriveMethods extends LinearOpMode {
//    @Override
//    public void runOpMode() {}
//
//    public void initOdometry(Pose2d pos) {
//        m_frontLeftLocation = new Translation2d(0.381, 0.381);
//        m_frontRightLocation = new Translation2d(0.381, -0.381);
//        m_backLeftLocation = new Translation2d(-0.381, 0.381);
//        m_backRightLocation = new Translation2d(-0.381, -0.381);
//        m_kinematics = new MecanumDriveKinematics(
//                        m_frontLeftLocation, m_frontRightLocation,
//                        m_backLeftLocation, m_backRightLocation
//                );
//
//        m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d(0.0), pos);
//
//        od_pose = m_odometry.getPoseMeters();
//    }
//
//    public void updateOdometry() {
//
//    }
//    public void initOdometry() {
//        initOdometry(new Pose2d(5.0,13.5,new Rotation2d(0)));
//    }
//}
