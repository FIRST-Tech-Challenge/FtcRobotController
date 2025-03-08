package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="Blue Sample Auto", group = "Autonomous")
public class BlueSampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double halfWidth = 7.4375;
        double halfLength = 8.125;
        Pose2d initialPose = new Pose2d(47.8 - halfWidth, 72 - halfLength, Math.toRadians(270));

        // Use RR drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // mechanism initialization
        Lift lift3 = new Lift(hardwareMap);
        SlideIntake slideIntake1 = new SlideIntake(hardwareMap);
        RobotServos servos   = new RobotServos(hardwareMap);

        TrajectoryActionBuilder initialDrive = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(53 - halfWidth, 63 - halfLength), null, new ProfileAccelConstraint(-80, 80))
                ;

        // building trajectories
        TrajectoryActionBuilder driveToBucket = drive.actionBuilder(new Pose2d(53 - halfWidth, 63 - halfLength, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(63 - halfWidth, 63 -  halfLength, Math.toRadians(30)),Math.toRadians(30), null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder driveIntoBucket = drive.actionBuilder(new Pose2d(63 - halfWidth, 63 - halfLength, Math.toRadians(30)))
                .splineToLinearHeading(new Pose2d(67 - halfWidth, 67 -  halfLength, Math.toRadians(30)),Math.toRadians(30), null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder driveBackUp = drive.actionBuilder(new Pose2d(67 - halfWidth, 67 - halfLength, Math.toRadians(30)))
                .splineToLinearHeading(new Pose2d(63 - halfWidth, 63 -  halfLength, Math.toRadians(30)),Math.toRadians(30), null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder driveBack = drive.actionBuilder(new Pose2d(63 - halfWidth, 63 - halfLength, Math.toRadians(30)))
                .splineToLinearHeading(new Pose2d(58 - halfWidth, 52.5 - halfLength, Math.toRadians(78)),  Math.toRadians(78),null, new ProfileAccelConstraint(-80, 80))
                ; //-1170

        TrajectoryActionBuilder driveToBucket2 = drive.actionBuilder(new Pose2d(57 - halfWidth, 52 - halfLength, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(63 - halfWidth, 63 -  halfLength, Math.toRadians(40)),Math.toRadians(40), null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder driveIntoBucket2 = drive.actionBuilder(new Pose2d(63 - halfWidth, 63 - halfLength, Math.toRadians(30)))
                .splineToLinearHeading(new Pose2d(67 - halfWidth, 67 -  halfLength, Math.toRadians(40)),Math.toRadians(40), null, new ProfileAccelConstraint(-80, 80));

        /* TrajectoryActionBuilder driveBack2 = drive.actionBuilder(new Pose2d(67 - halfWidth, 67 - halfLength, Math.toRadians(25)))
                .lineToY(62 - halfLength)
                ;  */

        TrajectoryActionBuilder driveBackUp2 = drive.actionBuilder(new Pose2d(67 - halfWidth, 67 - halfLength, Math.toRadians(40)))
                .splineToLinearHeading(new Pose2d(63 - halfWidth, 63 -  halfLength, Math.toRadians(40)),Math.toRadians(40), null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder driveBack2 = drive.actionBuilder(new Pose2d(63 - halfWidth, 63 - halfLength, Math.toRadians(40)))
                .lineToY(60 - halfLength)
                .splineToLinearHeading(new Pose2d(56.5 - halfWidth, 50 - halfLength, Math.toRadians(87)),  Math.toRadians(87),null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(71 - halfWidth, 49.5 - halfLength))
                ; //-1170

        TrajectoryActionBuilder driveToBucket3 = drive.actionBuilder(new Pose2d(73 - halfWidth, 50 - halfLength, Math.toRadians(90)))
                .strafeTo(new Vector2d(56.5 - halfWidth, 54 - halfLength))
                .splineToLinearHeading(new Pose2d(63 - halfWidth, 63 -  halfLength, Math.toRadians(40)),Math.toRadians(40), null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder driveIntoBucket3 = drive.actionBuilder(new Pose2d(63 - halfWidth, 63 - halfLength, Math.toRadians(30)))
                .splineToLinearHeading(new Pose2d(66.5 - halfWidth, 66.5 -  halfLength, Math.toRadians(40)),Math.toRadians(40), null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder driveBack3 = drive.actionBuilder(new Pose2d(67 - halfWidth, 67 - halfLength, Math.toRadians(40)))
                .splineToLinearHeading(new Pose2d(63 - halfWidth, 63 -  halfLength, Math.toRadians(40)),Math.toRadians(40), null, new ProfileAccelConstraint(-80, 80));

        // creating actions
        Action trajectoryBucketAction = driveToBucket.build();
        Action trajectoryBucketAction2 = driveToBucket2.build();
        Action trajectoryBucketAction3 = driveToBucket3.build();
        Action trajectoryBucketIn = driveIntoBucket.build();
        Action trajectoryBucketIn2 = driveIntoBucket2.build();
        Action trajectoryBucketIn3 = driveIntoBucket3.build();
        Action trajectoryBack = driveBack.build();
        Action trajectoryBack2 = driveBack2.build();
        Action trajectoryBack3= driveBack3.build();
        Action trajectoryBackUp = driveBackUp.build();
        Action trajectoryBackUp2 = driveBackUp2.build();
        Action liftToHighBox = lift3.liftUp(4800);
        Action liftToHighBox2 = lift3.liftUp(4800);
        Action liftToHighBox3 = lift3.liftUp(4800);
        Action liftDown = lift3.liftDown(0);
        Action liftDown2 = lift3.liftDown(0);
        Action liftDown3 = lift3.liftDown(0);
        Action openTopClaw = servos.moveTopClaw(0.0);
        Action openTopClaw2 = servos.moveTopClaw(0.0);
        Action openTopClaw3 = servos.moveTopClaw(0.0);
        Action flipTClawOut = servos.moveFlipTClaw(0.47); // for sample
        Action flipTClawOut2 = servos.moveFlipTClaw(0.47); // for sample
        Action flipTClawOut3 = servos.moveFlipTClaw(0.47); // for sample
        Action rotateTClaw = servos.moveRotateTClaw(1); // for sample
        Action rotateTClaw2 = servos.moveRotateTClaw(1); // for sample
        Action rotateTClaw3 = servos.moveRotateTClaw(1); // for sample
        Action closeTopClaw = servos.moveTopClaw(1.0);
        Action closeTopClaw2 = servos.moveTopClaw(1.0);
        Action closeTopClaw3 = servos.moveTopClaw(1.0);
        Action closeBottomClaw = servos.moveBottomClaw(1.0);
        Action closeBottomClaw2 = servos.moveBottomClaw(1.0);
        Action openBottomClaw = servos.moveBottomClaw(0.0);
        Action openBottomClaw2 = servos.moveBottomClaw(0.0);
        Action openBottomClaw3 = servos.moveBottomClaw(0.0);
        Action openBottomClaw4 = servos.moveBottomClaw(0.0);
        Action rotateArmOut    = servos.moveRotateArm(1.0);
        Action rotateArmOut2    = servos.moveRotateArm(1.0);
        Action rotateArmIn    = servos.moveRotateArm(0.0);
        Action rotateArmIn2    = servos.moveRotateArm(0.0);
        Action slideIntakeOut = slideIntake1.slideOut(-400);
        Action slideIntakeOut2 = slideIntake1.slideOut(-400);
        Action slideIntakeIn = slideIntake1.slideIn(-120);
        Action slideIntakeIn2 = slideIntake1.slideIn(-120);
        Action slideIntakeIn3 = slideIntake1.slideIn(0);
        Action rotateBClaw = servos.moveRotateBClaw(0.37);
        Action flipTClawIn = servos.moveFlipTClaw(1.0);
        Action flipTClawIn2 = servos.moveFlipTClaw(1.0);
        Action rotateTClawIn = servos.moveRotateTClaw(0.65);
        Action rotateTClawIn2 = servos.moveRotateTClaw(0.65);
        Action openTClaw = servos.moveTopClaw(0.45);
        Action openTClaw2 = servos.moveTopClaw(0.45);
        //Action slideIntakeOut = slideIntake2.slideMoveAction(-1079, 0.7);
        //Action slideInTakeIn = slideIntake2.slideMoveAction((int)slideIntakeStartPos, 0.7);

        Actions.runBlocking(closeTopClaw);

        while (!isStarted() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Autonomous", "Started");

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    rotateBClaw,
                                    flipTClawOut,
                                    rotateTClaw,
                                    liftToHighBox,
                                    trajectoryBucketAction
                            ),
                            new SequentialAction(
                                    trajectoryBucketIn,
                                    openTopClaw,
                                    new SleepAction(0.5),
                                    trajectoryBackUp
                            ),
                            new ParallelAction(
                                    trajectoryBack,
                                    liftDown,
                                    slideIntakeOut
                            ),
                            new SequentialAction(
                                    openBottomClaw,
                                    rotateArmOut,
                                    new SleepAction(1.0),
                                    closeBottomClaw,
                                    new SleepAction(0.5),
                                    slideIntakeIn,
                                    new SleepAction(0.5),
                                    rotateArmIn,
                                    new SleepAction(0.7),
                                    openTClaw,
                                    rotateTClawIn,
                                    flipTClawIn,
                                    new SleepAction(1.0),
                                    closeTopClaw2,
                                    new SleepAction(0.5),
                                    openBottomClaw2,
                                    new SleepAction(0.5),
                                    flipTClawOut2,
                                    rotateTClaw2
                            ),
                            new ParallelAction(
                                    liftToHighBox2,
                                    trajectoryBucketAction2
                            ),
                            new SequentialAction(
                                    trajectoryBucketIn2,
                                    openTopClaw2,
                                    new SleepAction(0.5),
                                    trajectoryBackUp2
                            ),
                            new ParallelAction(
                                    trajectoryBack2,
                                    liftDown2,
                                    slideIntakeOut2
                            ),
                            new SequentialAction(
                                    openBottomClaw3,
                                    rotateArmOut2,
                                    new SleepAction(1.0),
                                    closeBottomClaw2,
                                    new SleepAction(0.5),
                                    slideIntakeIn2,
                                    new SleepAction(0.5),
                                    rotateArmIn2,
                                    new SleepAction(0.7),
                                    openTClaw2,
                                    rotateTClawIn2,
                                    flipTClawIn2,
                                    new SleepAction(1.0),
                                    closeTopClaw3,
                                    new SleepAction(0.5),
                                    openBottomClaw4,
                                    new SleepAction(0.5),
                                    flipTClawOut3,
                                    rotateTClaw3
                            ),
                            new ParallelAction(
                                    liftToHighBox3,
                                    trajectoryBucketAction3
                            ),
                            new SequentialAction(
                                    trajectoryBucketIn3,
                                    openTopClaw3
                            ),
                            new SequentialAction(
                                    trajectoryBack3,
                                    liftDown3,
                                    slideIntakeIn3
                            )
                            /*openBottomClaw,
                            slideIntakeOut,
                            rotateArmOut,
                            new SleepAction(1),
                            closeBottomClaw,
                            new SleepAction(0.5),
                            rotateArmIn,
                            slideInTakeIn))*/));

        }
    }
}
