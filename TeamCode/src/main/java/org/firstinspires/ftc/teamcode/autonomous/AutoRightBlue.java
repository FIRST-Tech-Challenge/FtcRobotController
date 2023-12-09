package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.Robot;

@Autonomous(name="Autonomous Right Blue")
public class AutoRightBlue extends AutoBase {
    @Override
    public void runAutonomous(Robot robot, SampleMecanumDrive drive, TeamPropDetection.propLocation propLoc) {
        myLocalizer.setPoseEstimate(c.preStartPoseBlueRight);
        drive.setPoseEstimate(c.preStartPoseBlueRight); // !!!!!

        Pose2d teamPropCoordinate;
        if (propLoc == TeamPropDetection.propLocation.LEFT) {
            teamPropCoordinate = c.leftTeamPropBlueRight;
        }
        else if (propLoc == TeamPropDetection.propLocation.CENTER) {
            teamPropCoordinate = c.centerTeamPropBlueRight;
        }
        else if (propLoc == TeamPropDetection.propLocation.RIGHT) {
            teamPropCoordinate = c.rightTeamPropBlueRight;
        }
        else {
            teamPropCoordinate = c.rightTeamPropBlueRight;
        }
        // hardware map to get motors and sensors
        TrajectorySequence dropPropPixelRight = drive.trajectorySequenceBuilder(c.preStartPoseBlueRight)
                //.lineTo(c.leftTeamProp)
                //.lineTo(c.centerTeamProp)
                .lineToLinearHeading(c.startPoseBlueRight)
                .lineToLinearHeading(teamPropCoordinate)
                .back(3.5)
                .forward(3)
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(dropPropPixelRight.end())
                .lineToLinearHeading(c.leftBackdropIntermediateCenter)
                .lineToLinearHeading(c.leftBackdropLeft, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        robot.closeClaw = true;
        robot.updateSync();
        // Test propLoc here
        drive.followTrajectorySequence(dropPropPixelRight);
//
//        robot.outtakePixels = true;
//        robot.updateSync();
//        drive.followTrajectorySequence(goToBackdrop);
//        robot.closeClaw = false;
//        robot.updateSync();
    }
}
