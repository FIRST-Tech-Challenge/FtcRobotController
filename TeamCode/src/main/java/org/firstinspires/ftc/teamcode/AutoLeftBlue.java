package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.Robot;

@Autonomous(name="Autonomous Left Blue")
public class AutoLeftBlue extends AutoBase {
    @Override
    public void runAutonomous(Robot robot, SampleMecanumDrive drive, TeamPropDetection.propLocation propLoc) {
        myLocalizer.setPoseEstimate(c.preStartPoseBlueLeft);
        drive.setPoseEstimate(c.preStartPoseBlueLeft); // !!!!!

        Pose2d teamPropCoordinate;
        Pose2d backdropCoordinate;
        if (propLoc == TeamPropDetection.propLocation.LEFT) {
            teamPropCoordinate = c.leftTeamPropBlueLeft;
            backdropCoordinate = c.leftBackdropLeft;
        }
        else if (propLoc == TeamPropDetection.propLocation.CENTER) {
            teamPropCoordinate = c.centerTeamPropBlueLeft;
            backdropCoordinate = c.leftBackdropCenter;
        }
        else {
            teamPropCoordinate = c.rightTeamPropBlueLeft;
            backdropCoordinate = c.leftBackdropRight;
        }

        // hardware map to get motors and sensors
        TrajectorySequence dropPropPixelRight = drive.trajectorySequenceBuilder(c.preStartPoseBlueLeft)
                //.lineTo(c.leftTeamProp)
                //.lineTo(c.centerTeamProp)
                .lineToLinearHeading(c.startPoseBlueLeft)
                .lineToLinearHeading(teamPropCoordinate)
                .back(3.5)
                .forward(3)
                .lineTo(new Vector2d(-32, 11))
                .lineTo(new Vector2d(-50, 11))
                .lineTo(new Vector2d(-50, 35))
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(dropPropPixelRight.end())
                .lineToLinearHeading(c.leftBackdropIntermediateCenter)
                .lineToLinearHeading(backdropCoordinate, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .forward(8)
                .lineToLinearHeading(c.rightParkIntermediateBlueLeft)
                .lineToLinearHeading(c.rightParkFinalBlueLeft)
                .build();

        robot.closeClaw = true;
        robot.updateSync();
        // Test propLoc here
        drive.followTrajectorySequence(dropPropPixelRight);

        robot.outtakePixels = true;
        robot.updateSync();
        drive.followTrajectorySequence(goToBackdrop);
        robot.closeClaw = false;
        robot.updateSync();
        drive.followTrajectorySequence(parkRight);
    }
}
