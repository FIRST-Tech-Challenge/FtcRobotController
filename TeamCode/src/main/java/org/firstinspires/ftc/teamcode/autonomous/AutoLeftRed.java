package org.firstinspires.ftc.teamcode.autonomous;

// imports

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.Robot;


@Autonomous(name="Autonomous Left Red")
public class AutoLeftRed extends AutoBase {
    @Override
    public void runAutonomous(Robot robot, SampleMecanumDrive drive, TeamPropDetection.propLocation propLoc) {
        myLocalizer.setPoseEstimate(c.preStartPoseRedLeft);
        drive.setPoseEstimate(c.preStartPoseRedLeft); // !!!!!

        Pose2d teamPropCoordinate;
        if (propLoc == TeamPropDetection.propLocation.LEFT) {
            teamPropCoordinate = c.leftTeamPropRedLeft;
        }
        else if (propLoc == TeamPropDetection.propLocation.CENTER) {
            teamPropCoordinate = c.centerTeamPropRedLeft;
        }
        else if (propLoc == TeamPropDetection.propLocation.RIGHT) {
            teamPropCoordinate = c.rightTeamPropRedLeft;
        }
        else {
            teamPropCoordinate = c.rightTeamPropRedLeft;
        }
        // hardware map to get motors and sensors
        TrajectorySequence dropPropPixelRight = drive.trajectorySequenceBuilder(c.preStartPoseRedLeft)
                //.lineTo(c.leftTeamProp)
                //.lineTo(c.centerTeamProp)
                .lineToLinearHeading(c.startPoseRedLeft)
                .lineToLinearHeading(teamPropCoordinate)
                .back(3.5)
                .forward(3)
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(dropPropPixelRight.end())
                .lineToLinearHeading(c.rightBackdropIntermediateCenter)
                .lineToLinearHeading(c.rightBackdropLeft, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
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
