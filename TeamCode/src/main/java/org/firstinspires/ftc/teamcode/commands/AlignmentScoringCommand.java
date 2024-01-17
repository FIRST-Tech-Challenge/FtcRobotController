package org.firstinspires.ftc.teamcode.commands;

<<<<<<< Updated upstream
import com.arcrobotics.ftclib.command.CommandBase;

public class AlignmentScoringCommand extends CommandBase {

}
=======
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.subsystems.AprilVision2;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AlignmentScoringCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final AprilVision2 m_aprilVision;
    private boolean m_isFieldCentric;
    public static PIDCoefficients m_xCoeffiecents = new PIDCoefficients(1, 0.0, 0.1);
    public static PIDCoefficients m_yCoeffiecents = new PIDCoefficients(1, 0.0, 0.1);
    public static PIDCoefficients m_thetaCoeffiecents = new PIDCoefficients(1, 0.0, 0.1);
    public static int m_position = 0;

    final double DESIRED_DISTANCE =
            4.0; //  this is how close the camera should get to the target (inches)
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN  =  0.02 ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN   =  0.012  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.65;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 0.65;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.35;   //  Clip the turn speed to this max value (adjust for your robot)


    double drive;
    double strafe;
    double turn;

    double rangeError;

    int m_tagID;

    Boolean backCamera;

    public AlignmentScoringCommand(DriveSubsystem drive, AprilVision2 vision, boolean isFieldCentric, int desiredTag, Boolean camera) {

        m_driveSubsystem = drive;
        m_isFieldCentric = isFieldCentric;
         m_aprilVision = vision;
        m_tagID = desiredTag;
        addRequirements(m_driveSubsystem);
        addRequirements(m_aprilVision);
        backCamera = camera;

    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(backCamera == true) {
            if (m_aprilVision.seeingAprilTags()) {
                AprilTagDetection desiredTag = m_aprilVision.getDesiredAprilTag(m_tagID);
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                if (desiredTag != null) {
                    rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(-rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                }
            } else {
            }


            // Apply desired axes motions to the drivetrain.
            m_driveSubsystem.moveRobot(drive, strafe, turn);
        } else {
            if (m_aprilVision.seeingAprilTags()) {
                AprilTagDetection desiredTag = m_aprilVision.getDesiredAprilTag(m_tagID);
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                if (desiredTag != null) {
                    rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                }
            } else {
            }


            // Apply desired axes motions to the drivetrain.
            m_driveSubsystem.moveRobot(-drive, -strafe, -turn);


        }

    }
    @Override
    public boolean isFinished(){
        if(drive < 0.05 && strafe < 0.05 && turn < 0.05 && rangeError < 3.55){
            return true;
        }

        return false;
    }
    public void periodic(){
        m_driveSubsystem.updatePoseEstimate();
    }
}

>>>>>>> Stashed changes
