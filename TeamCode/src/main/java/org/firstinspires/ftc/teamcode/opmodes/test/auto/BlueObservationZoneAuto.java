package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Arm;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Claw;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.LinearSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.XYaw;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.YPitch;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "Blue Observation Zone Auto", group = "Jan Final")
@Disabled
public class BlueObservationZoneAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the MecanumDrive with the hardware map
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, 60, Math.toRadians(-90)));
        Arm arm = new Arm(hardwareMap);
        LinearSlide linearSlide = new LinearSlide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        XYaw yaw = new XYaw(hardwareMap);
        YPitch pitch = new YPitch(hardwareMap);

        /** Overall Steps **/
        // Initial pose near observation area facing the submersible
        // Game begins with one sample with the robot
        // Turn towards the observation area
        // Go towards the observation area, can enter the area, make sure the human doesn't have his hand in there at that time
        // Drop the sample
        // Strafe towards the first sample (closest to the submersible)
        // Strafe back towards the observation area
        // Pick the sample
        // Drop the sample
        // Strafe towards the second sample (middle one among three samples)
        // Pick the sample
        // Drop the sample
        // Strafe towards the observation area
        // Turn towards the observation area
        // Drop the sample
        // Strafe towards the third sample (one by the wall)
        // Turn towards the sample
        // Pick the sample
        // Strafe towards the observation area
        // Turn
        // Drop the sample
        // Pick the specimen
        // Spline towards the short side of the submersible
        // Turn
        // Clip the sample on the high bar/ low bar
        // Turn
        // Move towards the observation area
        // Pick the specimen
        // Spline towards the short side of the submersible
        // Turn
        // Clip the sample on the high bar/ low bar
        // Turn
        // Move towards the observation area
        // Pick the specimen
        // Spline towards the short side of the submersible
        // Turn
        // Clip the sample on the high bar/ low bar
        // Turn
        // Move towards the observation area
        // Pick the specimen
        // Spline towards the short side of the submersible
        // Turn
        // Clip the sample on the high bar/ low bar
        // Turn
        // Move towards the long side of the submersible
        // Hang
        /** Overall Steps **/


        /** Trajectory 1 **/
        // Initial pose near observation area facing the submersible
        // Game begins with one sample with the robot
        // Turn towards the observation area
        // Go towards the observation area, can enter the area, make sure the human doesn't have his hand in there at that time

        /** Action 1 **/
        // Drop the sample

        /** Trajectory 2 **/
        // Strafe towards the first sample (closest to the submersible)

        /** Action 2 **/
        // Pick the sample

        /** Trajectory 3 **/
        // Strafe back towards the observation area

        /** Action 3 **/
        // Drop the sample

        /** Trajectory 4 **/
        // Strafe towards the second sample (middle one among three samples)

        /** Action 4 **/
        // Pick the sample

        /** Trajectory 5 **/
        // Strafe towards the observation area
        // Turn towards the observation area

        /** Action 5 **/
        // Drop the sample

        /** Trajectory 6 **/
        // Strafe towards the third sample (one by the wall)
        // Turn towards the sample

        /** Action 6 **/
        // Pick the sample

        /** Trajectory 7 **/
        // Strafe towards the observation area
        // Turn

        /** Action 7 **/
        // Drop the sample

        /** Action 8 **/
        // Pick the sample

        /** Trajectory 8 **/
        // Spline towards the short side of the submersible
        // Turn

        /** Action 9 **/
        // Clip the sample on the high bar/ low bar

        /** Trajectory 9 **/
        // Turn
        // Move towards the observation area

        /** Action 10 **/
        // Pick the specimen

        /** Trajectory 10 **/
        // Spline towards the short side of the submersible
        // Turn

        /** Action 11 **/
        // Clip the sample on the high bar/ low bar

        /** Trajectory 11 **/
        // Turn
        // Move towards the observation area

        /** Action 12 **/
        // Pick the specimen

        /** Trajectory 12 **/
        // Spline towards the short side of the submersible
        // Turn

        /** Action 13 **/
        // Clip the sample on the high bar/ low bar

        /** Trajectory 13 **/
        // Turn
        // Move towards the observation area

        /** Action 14 **/
        // Pick the specimen

        /** Trajectory 14 **/
        // Spline towards the short side of the submersible
        // Turn

        /** Action 15 **/
        // Clip the sample on the high bar/ low bar

        /** Trajectory 15 **/
        // Turn
        // Move towards the long side of the submersible

        /** Action 16 **/
        // Hang

        /** Trajectory 1 **/
        // Initial pose near observation area facing the submersible
        // Game begins with one sample with the robot
        // Turn towards the observation area
        // Go towards the observation area, can enter the area, make sure the human doesn't have his hand in there at that time
        double xPositionInit = -16.0;
        double yPositionInit = 58.0;
        double headingInit = Math.toRadians(180.0);

        Pose2d initialPose = new Pose2d(xPositionInit, yPositionInit, headingInit);

        double xDestPositionDropSampleInHand = -36.0;
        double yDestPositionDropSampleInHand = yPositionInit;
        double headingDestPositionDropSampleInHand = Math.toRadians(180.0);

        TrajectoryActionBuilder initToObservationZone = drive.actionBuilder(initialPose)
//                .turnTo(Math.toRadians(headingDestPositionDropSampleInHand));
                .lineToX(xDestPositionDropSampleInHand);

        /** Action 1 **/
        // Drop the sample

        /** Trajectory 2 **/
        // Strafe towards the first sample (closest to the submersible)
        double xDestPositionPickSample1 = xDestPositionDropSampleInHand;
        double yDestPositionPickSample1 = 24.0;
        double headingDestPositionPickSample1 = headingDestPositionDropSampleInHand;

        Pose2d initPosePickSample1 = new Pose2d(xDestPositionDropSampleInHand, yDestPositionDropSampleInHand, headingDestPositionDropSampleInHand);

        Vector2d targetVectorPickSample1 = new Vector2d(xDestPositionPickSample1, yDestPositionPickSample1);

        TrajectoryActionBuilder obsAreaToPickSample1 = drive.actionBuilder(initPosePickSample1)
                .strafeTo(targetVectorPickSample1);

        /** Action 2 **/
        // Pick the sample

        /** Trajectory 3 **/
        // Strafe back towards the observation area
        double xDestPositionDropSample1 = xDestPositionDropSampleInHand;
        double yDestPositionDropSample1 = yDestPositionDropSampleInHand;
        double headingDestPositionDropSample1 = headingDestPositionDropSampleInHand;

        Pose2d initPoseDropSample1 = new Pose2d(xDestPositionPickSample1, yDestPositionPickSample1, headingDestPositionPickSample1);

        Vector2d targetVectorDropSample1 = new Vector2d(xDestPositionDropSample1, yDestPositionDropSample1);

        TrajectoryActionBuilder sample1PickToDropSample1 = drive.actionBuilder(initPoseDropSample1)
                .strafeTo(targetVectorDropSample1);

        /** Action 3 **/
        // Drop the sample

        /** Trajectory 4 **/
        // Strafe towards the second sample (middle one among three samples)
        double xDestPositionPickSample2 = xDestPositionPickSample1 - 10.0;
        double yDestPositionPickSample2 = yDestPositionPickSample1;
        double headingDestPositionPickSample2 = headingDestPositionPickSample1;

        Pose2d initPosePickSample2 = new Pose2d(xDestPositionDropSample1, yDestPositionDropSample1, headingDestPositionDropSample1);

        Vector2d targetVectorPickSample2 = new Vector2d(xDestPositionPickSample2, yDestPositionPickSample2);

        TrajectoryActionBuilder obsAreaToPickSample2 = drive.actionBuilder(initPosePickSample2)
                .strafeTo(targetVectorPickSample2);


        /** Action 4 **/
        // Pick the sample

        /** Trajectory 5 **/
        // Strafe towards the observation area
        // Turn towards the observation area
        double xDestPositionDropSample2 = xDestPositionDropSample1 - 10.0;
        double yDestPositionDropSample2 = 48.0;
        double headingDestPositionDropSample2 = headingDestPositionDropSample1;
        double turnAngleDestPositionDropSample2 = Math.toRadians(135.0);

        Pose2d initPoseDropSample2 = new Pose2d(xDestPositionPickSample2, yDestPositionPickSample2, headingDestPositionPickSample2);

        Vector2d targetVectorDropSample2 = new Vector2d(xDestPositionDropSample2, yDestPositionDropSample2);

        TrajectoryActionBuilder sample2PickToDropSample2 = drive.actionBuilder(initPoseDropSample2)
                .strafeTo(targetVectorDropSample2)
                .turnTo(turnAngleDestPositionDropSample2);

        /** Action 5 **/
        // Drop the sample

        /** Trajectory 6 **/
        // Strafe towards the third sample (one by the wall)
        // Turn towards the sample
        double xDestPositionPickSample3 = xDestPositionDropSample2 - 6.0;
        double yDestPositionPickSample3 = yDestPositionPickSample2;
        double headingDestPositionPickSample3 = Math.toRadians(180.0);

        Pose2d initPosePickSample3 = new Pose2d(xDestPositionDropSample2, yDestPositionDropSample2, turnAngleDestPositionDropSample2);

        Vector2d targetVectorPickSample3 = new Vector2d(xDestPositionPickSample3, yDestPositionPickSample3);

        TrajectoryActionBuilder obsAreaToPickSample3 = drive.actionBuilder(initPosePickSample3)
                .strafeTo(targetVectorPickSample3)
                .turnTo(headingDestPositionPickSample3);

        /** Action 6 **/
        // Pick the sample

        /** Trajectory 7 **/
        // Strafe towards the observation area
        // Turn
        double xDestPositionDropSample3 = xDestPositionDropSample2 - 6.0;
        double yDestPositionDropSample3 = 48.0;
        double headingDestPositionDropSample3 = headingDestPositionDropSample2;
        double turnAngleDestPositionDropSample3 = Math.toRadians(90.0);

        Pose2d initPoseDropSample3 = new Pose2d(xDestPositionPickSample3, yDestPositionPickSample3, headingDestPositionPickSample3);

        Vector2d targetVectorDropSample3 = new Vector2d(xDestPositionDropSample3, yDestPositionDropSample3);

        TrajectoryActionBuilder sample3PickToDropSample3 = drive.actionBuilder(initPoseDropSample3)
                .strafeTo(targetVectorDropSample3)
                .turnTo(turnAngleDestPositionDropSample3);

        /** Action 7 **/
        // Drop the sample

        /** Action 8 **/
        // Pick the sample

        /** Trajectory 8 **/
        // Spline towards the short side of the submersible
        // Turn
        double xDestPositionHangSpecimen1 = 0.0;
        double yDestPositionHangSpecimen1 = 38.0;
        double turnAngleDestPositionHangSpecimen1 = Math.toRadians(-12.0);
        double headingDestPositionHangSpecimen1 = Math.toRadians(-90.0);

        Pose2d initPoseHangSpecimen1 = new Pose2d(xDestPositionDropSample3, yDestPositionDropSample3, turnAngleDestPositionDropSample3);

        Vector2d targetVectorHangSpecimen1 = new Vector2d(xDestPositionHangSpecimen1, yDestPositionHangSpecimen1);

        Pose2d targePoseHangSpecimen1 = new Pose2d(targetVectorHangSpecimen1, headingDestPositionHangSpecimen1);

        TrajectoryActionBuilder obsAreaToHangSpecimen1 = drive.actionBuilder(initPoseHangSpecimen1)
                .turnTo(turnAngleDestPositionHangSpecimen1)
                .lineToX(xDestPositionHangSpecimen1)
                .turnTo(headingDestPositionHangSpecimen1);
//                .turnTo(headingDestPositionHangSpecimen1)
//                .splineToConstantHeading(targetVectorHangSpecimen1, 0);
//                .strafeTo(targetVectorHangSpecimen1)
//                .turnTo(headingDestPositionHangSpecimen1);

        /** Action 9 **/
        // Clip the sample on the high bar/ low bar

        /** Trajectory 9 **/
        // Turn
        // Move towards the observation area
        double xDestPositionPickSpecimen2 = -32.0;
        double yDestPositionPickSpecimen2 = 56.0;
        double headingDestPositionPickSpecimen2 = Math.toRadians(-210);

        Pose2d initPosePickSpecimen2 = new Pose2d(xDestPositionHangSpecimen1, yDestPositionHangSpecimen1, headingDestPositionHangSpecimen1);

        Vector2d targetVectorPickSpecimen2 = new Vector2d(xDestPositionPickSpecimen2, yDestPositionPickSpecimen2);

        Pose2d targetPosePickSpecimen2 = new Pose2d(xDestPositionPickSpecimen2, yDestPositionPickSpecimen2, headingDestPositionPickSpecimen2);

        TrajectoryActionBuilder hangAreaToPickSpecimen2 = drive.actionBuilder(initPosePickSpecimen2)
                .turnTo(headingDestPositionPickSpecimen2)
                .lineToX(xDestPositionPickSpecimen2);
//                        .splineToLinearHeading(targetPosePickSpecimen2, Math.toRadians(90.0));
//                .strafeTo(targetVectorPickSpecimen2)
//                .turnTo(headingDestPositionPickSpecimen2);

        /** Action 10 **/
        // Pick the specimen

        /** Trajectory 10 **/
        // Spline towards the short side of the submersible
        // Turn
        double xDestPositionHangSpecimen2 = 0.0;
        double yDestPositionHangSpecimen2 = 38.0;
        double turnAngleDestPositionHangSpecimen2 = Math.toRadians(-30.0);
        double headingDestPositionHangSpecimen2 = Math.toRadians(-90.0);

        Pose2d initPoseHangSpecimen2 = new Pose2d(xDestPositionPickSpecimen2, yDestPositionPickSpecimen2, headingDestPositionPickSpecimen2);

        Vector2d targetVectorHangSpecimen2 = new Vector2d(xDestPositionHangSpecimen2, yDestPositionHangSpecimen2);

        Pose2d targePoseHangSpecimen2 = new Pose2d(targetVectorHangSpecimen2, headingDestPositionHangSpecimen2);

        TrajectoryActionBuilder obsAreaToHangSpecimen2 = drive.actionBuilder(initPoseHangSpecimen2)
                .turnTo(turnAngleDestPositionHangSpecimen2)
                .lineToX(xDestPositionHangSpecimen2)
                .turnTo(headingDestPositionHangSpecimen2);

        /** Action 11 **/
        // Clip the sample on the high bar/ low bar

        /** Trajectory 11 **/
        // Turn
        // Move towards the observation area
        double xDestPositionPickSpecimen3 = -32.0;
        double yDestPositionPickSpecimen3 = 56.0;
        double headingDestPositionPickSpecimen3 = Math.toRadians(-210);

        Pose2d initPosePickSpecimen3 = new Pose2d(xDestPositionHangSpecimen2, yDestPositionHangSpecimen2, headingDestPositionHangSpecimen2);

        Vector2d targetVectorPickSpecimen3 = new Vector2d(xDestPositionPickSpecimen3, yDestPositionPickSpecimen3);

        Pose2d targetPosePickSpecimen3 = new Pose2d(xDestPositionPickSpecimen3, yDestPositionPickSpecimen3, headingDestPositionPickSpecimen3);

        TrajectoryActionBuilder hangAreaToPickSpecimen3 = drive.actionBuilder(initPosePickSpecimen3)
                .turnTo(headingDestPositionPickSpecimen3)
                .lineToX(xDestPositionPickSpecimen3);

        /** Action 12 **/
        // Pick the specimen

        /** Trajectory 12 **/
        // Spline towards the short side of the submersible
        // Turn
        double xDestPositionHangSpecimen3 = 0.0;
        double yDestPositionHangSpecimen3 = 38.0;
        double turnAngleDestPositionHangSpecimen3 = Math.toRadians(-30.0);
        double headingDestPositionHangSpecimen3 = Math.toRadians(-90.0);

        Pose2d initPoseHangSpecimen3 = new Pose2d(xDestPositionPickSpecimen3, yDestPositionPickSpecimen3, headingDestPositionPickSpecimen3);

        Vector2d targetVectorHangSpecimen3 = new Vector2d(xDestPositionHangSpecimen3, yDestPositionHangSpecimen3);

        Pose2d targePoseHangSpecimen3 = new Pose2d(targetVectorHangSpecimen3, headingDestPositionHangSpecimen3);

        TrajectoryActionBuilder obsAreaToHangSpecimen3 = drive.actionBuilder(initPoseHangSpecimen3)
                .turnTo(turnAngleDestPositionHangSpecimen3)
                .lineToX(xDestPositionHangSpecimen3)
                .turnTo(headingDestPositionHangSpecimen3);

        /** Action 13 **/
        // Clip the sample on the high bar/ low bar

        /** Trajectory 13 **/
        // Turn
        // Move towards the observation area
        double xDestPositionPickSpecimen4 = -32.0;
        double yDestPositionPickSpecimen4 = 56.0;
        double headingDestPositionPickSpecimen4 = Math.toRadians(-210);

        Pose2d initPosePickSpecimen4 = new Pose2d(xDestPositionHangSpecimen3, yDestPositionHangSpecimen3, headingDestPositionHangSpecimen3);

        Vector2d targetVectorPickSpecimen4 = new Vector2d(xDestPositionPickSpecimen4, yDestPositionPickSpecimen4);

        Pose2d targetPosePickSpecimen4 = new Pose2d(xDestPositionPickSpecimen4, yDestPositionPickSpecimen4, headingDestPositionPickSpecimen4);

        TrajectoryActionBuilder hangAreaToPickSpecimen4 = drive.actionBuilder(initPosePickSpecimen4)
                .turnTo(headingDestPositionPickSpecimen4)
                .lineToX(xDestPositionPickSpecimen4);

        /** Action 14 **/
        // Pick the specimen

        /** Trajectory 14 **/
        // Spline towards the short side of the submersible
        // Turn
        double xDestPositionHangSpecimen4 = 0.0;
        double yDestPositionHangSpecimen4 = 38.0;
        double turnAngleDestPositionHangSpecimen4 = Math.toRadians(-30.0);
        double headingDestPositionHangSpecimen4 = Math.toRadians(-90.0);

        Pose2d initPoseHangSpecimen4 = new Pose2d(xDestPositionPickSpecimen4, yDestPositionPickSpecimen4, headingDestPositionPickSpecimen4);

        Vector2d targetVectorHangSpecimen4 = new Vector2d(xDestPositionHangSpecimen4, yDestPositionHangSpecimen4);

        Pose2d targePoseHangSpecimen4 = new Pose2d(targetVectorHangSpecimen4, headingDestPositionHangSpecimen4);

        TrajectoryActionBuilder obsAreaToHangSpecimen4 = drive.actionBuilder(initPoseHangSpecimen4)
                .turnTo(turnAngleDestPositionHangSpecimen4)
                .lineToX(xDestPositionHangSpecimen4)
                .turnTo(headingDestPositionHangSpecimen4);

        /** Action 15 **/
        // Clip the sample on the high bar/ low bar

        /** Trajectory 15 **/
        // Turn
        // Move towards the long side of the submersible
        double xDestPosition1HangRobot = -36.0;
        double yDestPosition1HangRobot = 36.0;
        double xDestPositionHangRobot = -36.0;
        double yDestPositionHangRobot = 0.0;
        double turnAngleDestPositionHangRobot = Math.toRadians(-30.0);
        double headingDestPositionHangRobot = Math.toRadians(180.0);
        double tangentDestPositionHangRobot = Math.toRadians(180.0);

        Pose2d initPoseHangRobot = new Pose2d(xDestPositionHangSpecimen4, yDestPositionHangSpecimen4, headingDestPositionHangSpecimen4);

        Vector2d target1VectorHangRobot = new Vector2d(xDestPosition1HangRobot, yDestPosition1HangRobot);
        Vector2d targetVectorHangRobot = new Vector2d(xDestPositionHangRobot, yDestPositionHangRobot);

        Pose2d targePoseHangRobot = new Pose2d(targetVectorHangRobot, headingDestPositionHangRobot);

        TrajectoryActionBuilder obsAreaToHangRobot = drive.actionBuilder(initPoseHangRobot)
//                .splineToSplineHeading(targePoseHangRobot, tangentDestPositionHangRobot);
                .strafeTo(target1VectorHangRobot)
                .turnTo(headingDestPositionHangRobot)
                .strafeTo(targetVectorHangRobot);

        /** Action 16 **/
        // Hang

        TrajectoryActionBuilder completeTrajectory = drive.actionBuilder(initialPose)
                .afterDisp(1, arm.raiseArmForLowerBasket())
                .afterDisp(1, yaw.moveWristCenter())
                .afterDisp(1, arm.initializeArm())
                .lineToX(xDestPositionDropSampleInHand)
                .afterDisp(1, arm.raiseArmForNetzone())
                .afterDisp(1, linearSlide.extendArmForward())
                .afterDisp(1, claw.openClaw())
                .afterDisp(1, linearSlide.retractSlideBackward())
                .strafeTo(targetVectorPickSample1)
//                .afterDisp(1, linearSlide.extendArmForward())
                .afterDisp(1, claw.closeClaw())
                .strafeTo(targetVectorDropSample1)
                .strafeTo(targetVectorPickSample2)
                .strafeTo(targetVectorDropSample2)
                .turnTo(turnAngleDestPositionDropSample2)
                .strafeTo(targetVectorPickSample3)
                .turnTo(headingDestPositionPickSample3)
                .strafeTo(targetVectorDropSample3)
                .turnTo(turnAngleDestPositionDropSample3)
                .turnTo(turnAngleDestPositionHangSpecimen1)
                .lineToX(xDestPositionHangSpecimen1)
                .turnTo(headingDestPositionHangSpecimen1)
                .turnTo(headingDestPositionPickSpecimen2)
                .lineToX(xDestPositionPickSpecimen2)
                .turnTo(turnAngleDestPositionHangSpecimen2)
                .lineToX(xDestPositionHangSpecimen2)
                .turnTo(headingDestPositionHangSpecimen2)
                .turnTo(headingDestPositionPickSpecimen3)
                .lineToX(xDestPositionPickSpecimen3)
                .turnTo(turnAngleDestPositionHangSpecimen3)
                .lineToX(xDestPositionHangSpecimen3)
                .turnTo(headingDestPositionHangSpecimen3)
                .turnTo(headingDestPositionPickSpecimen4)
                .lineToX(xDestPositionPickSpecimen4)
                .turnTo(turnAngleDestPositionHangSpecimen4)
                .lineToX(xDestPositionHangSpecimen4)
                .turnTo(headingDestPositionHangSpecimen4)
                .strafeTo(target1VectorHangRobot)
                .turnTo(headingDestPositionHangRobot)
                .strafeTo(targetVectorHangRobot);

        if (isStopRequested()) return;

        while (!isStopRequested() && !opModeIsActive()) {
            // Define the trajectory for the Blue Basket sequence with waits
//            Actions.runBlocking(new SequentialAction(
//                            initToObservationZone.build(),
//                            obsAreaToPickSample1.build(),
//                            sample1PickToDropSample1.build(),
//                            obsAreaToPickSample2.build(),
//                            sample2PickToDropSample2.build(),
//                            obsAreaToPickSample3.build(),
//                            sample3PickToDropSample3.build(),
//                            obsAreaToHangSpecimen1.build(),
//                            hangAreaToPickSpecimen2.build(),
//                            obsAreaToHangSpecimen2.build(),
//                            hangAreaToPickSpecimen3.build(),
//                            obsAreaToHangSpecimen3.build(),
//                            hangAreaToPickSpecimen4.build(),
//                            obsAreaToHangSpecimen4.build(),
//                            obsAreaToHangRobot.build()
//                    )
//            );

            Actions.runBlocking(new SequentialAction(completeTrajectory.build()));
        }
    }
}