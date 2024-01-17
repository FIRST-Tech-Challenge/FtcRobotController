package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AprilVision2;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class RelocalizeOffAprilTags extends CommandBase {
    AprilVision2 m_vision;
    DriveSubsystem m_drive;
    Telemetry m_telemetry;
    double AprilTag9X = 50;
    double AprilTag9Y = 36;
    double AprilTag9Z = Math.toRadians(180);
    double AprilTag7X = -72.6;
    double AprilTag7Y = 36;
    double AprilTag7Z = Math.toDegrees(0);


    double m_x;
    double m_y;
    double m_z;

    Boolean finished = false;

    Vector2d aprilTag9 = new Vector2d(60, 36);
    Vector2d aprilTag7 = new Vector2d(AprilTag7X, AprilTag7Y);

    Vector2d cameraPose = new Vector2d(3,0);

    public double headingOoffset = 0;

    public int desiredTag;
    public RelocalizeOffAprilTags(DriveSubsystem drive, AprilVision2 vision, Telemetry telemetry, int tag) {
        m_drive = drive;
        m_vision = vision;
        m_telemetry = telemetry;
        desiredTag = tag;
    }

    @Override
    public void execute() {
        if (m_vision.seeingAprilTags()) {
            Vector2d currentPose = new Vector2d();
            Vector2d computedPose =new Vector2d(0,0);
            if(desiredTag == 9) {
               // currentPose.rotateBy(180);
                //aprilTag9.rotateBy(180);
                computedPose = cameraPose.plus(aprilTag9.minus(currentPose));
                computedPose.rotateBy(180);
            }

            if(desiredTag == 7) {
                computedPose = cameraPose.plus(aprilTag7.plus(currentPose));
                headingOoffset = 180;
            }
            m_x = computedPose.getX();
            m_y = computedPose.getY();
            m_z = 0;
            if(m_x != -72.6) {
                m_drive.setPoseEstimate(new Pose2d(m_x, m_y, m_z));
                finished = true;
            }
            m_telemetry.addData("AprilRobotPose", new Pose2d(m_x, m_y, m_z));

        }
    }
    public boolean isFinished(){
        return finished;
    }

//    public void periodic() {
//        if (m_vision.seeingAprilTags()) {
//            Vector2d currentPose = new Vector2d(m_vision.getFtcPoseX(7), m_vision.getFtcPoseY(7));
//            Vector2d computedPose = aprilTag7.plus(currentPose);
//            m_x = computedPose.getX();
//            m_y = computedPose.getY();
//            m_z = m_vision.getPoseYaw(7) + m_vision.getPoseBearing(7);
//
//            m_telemetry.addData("AprilRobotPose", new Pose2d(m_x, m_y, m_z));
//
//        }
//    }
}
