package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "BlueSpecAuto", group = "Autonomous")
public class BlueSpecAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        double halfWidth = 7.4375;
        double halfLength = 8.125;

        Pose2d initialPose = new Pose2d(24 - halfWidth, -72 + halfLength, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Lift lift            = new Lift(hardwareMap);
        SlideIntake slide1   = new SlideIntake(hardwareMap);
        RobotServos servos   = new RobotServos(hardwareMap);
        Servo rotateArm    = hardwareMap.get(Servo.class, "rotateArm");


        /*int liftMotor1StartPosition = lift.liftMotor1.getCurrentPosition();
        int liftMotor1EndPosition = liftMotor1StartPosition + 4650; */


        Action liftToHighJunctionNew = lift.liftUp(1550);
        Action liftToHighJunction = lift.liftUp(3000);
        Action liftToHighJunction2 = lift.liftUp(3000);
        Action liftToHighJunction3 = lift.liftUp(3000);
        Action liftToLowPosition  = lift.liftDown( 1700);
        Action liftToLowPosition2  = lift.liftDown( 1700);
        Action liftToLowPosition3  = lift.liftDown( 1600);
        Action liftDown = lift.liftDown(0);
        Action liftDown2 = lift.liftDown(0);
        Action liftDown3 = lift.liftDown(0);
        Action openTopClaw2 = servos.moveTopClaw(0.3);
        Action openTopClaw3 = servos.moveTopClaw(0.3);
        Action flipTClawOut3 = servos.moveFlipTClaw(0); // for specimen


        Action slideIn  = slide1.slideIn( 109);
        Action slideOut = slide1.slideOut(-1250);

        Action openBottomClaw  = servos.moveBottomClaw(0.0);
        Action closeBottomClaw = servos.moveBottomClaw(1.0);
        Action closeTopClaw = servos.moveTopClaw(1.0);
        Action closeTopClaw2 = servos.moveTopClaw(1.0);
        Action closeTopClaw3 = servos.moveTopClaw(1.0);
        Action closeTopClaw4 = servos.moveTopClaw(1.0);

        Action openTopClaw = servos.moveTopClaw(0.3);
        Action rotateArmOut    = servos.moveRotateArm(1.0);
        Action flipTClawOut = servos.moveFlipTClaw(0.2); // for specimen
        Action flipTClawOut2 = servos.moveFlipTClaw(0.2); // for specimen
        Action flipTClawOut5 = servos.moveFlipTClaw(0.21); // for specimen
        Action flipTClawOutNew = servos.moveFlipTClaw(0.25); // for specimen

        Action flipTClawIn = servos.moveFlipTClaw(1.0);
        Action rotateTClaw = servos.moveRotateTClaw(0.88); // for specimen
        Action rotateTClaw2 = servos.moveRotateTClaw(0.88); // for specimen
        Action rotateTClawNew = servos.moveRotateTClaw(0.65); // for specimen

        Action flipTClawOut4 = servos.moveFlipTClaw(0.21);
        Action openTopClaw4 = servos.moveTopClaw(0.3);
        Action openTopClaw5 = servos.moveTopClaw(0.3);
        Action openTopClaw6 = servos.moveTopClaw(0.3);
        Action rotateTClaw3 = servos.moveRotateTClaw(0.88);

        ;
        TrajectoryActionBuilder drive1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-15 + halfWidth, -45 + halfLength));

        TrajectoryActionBuilder drive2 = drive.actionBuilder(new Pose2d(-7 + halfWidth, -45.5 + halfLength,Math.toRadians(90)))
                .lineToY(-50 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(47 - halfWidth, -45.5 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(51, -54 + halfLength, Math.toRadians(245)),Math.toRadians(245), null, new ProfileAccelConstraint(-20, 20))
                ;

        /*TrajectoryActionBuilder drive2 = drive.actionBuilder(new Pose2d(-12 + halfWidth, -44.5 + halfLength,Math.toRadians(90)))
                .lineToY(-50 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(47 - halfWidth, -45.5 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-22 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(53, -22 + halfLength),null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-52, null, new ProfileAccelConstraint(-80, 80))
                .lineToY(-22 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(63, -22 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-52, null, new ProfileAccelConstraint(-80, 80))
                .lineToY(-35, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(51,-35), null, new ProfileAccelConstraint(-80, 80))
                //.strafeTo(new Vector2d(52, -72 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(51, -55 + halfLength, Math.toRadians(235)),Math.toRadians(235), null, new ProfileAccelConstraint(-80, 80))
                ; */


        TrajectoryActionBuilder drive3 = drive.actionBuilder(new Pose2d(51, -54 + halfLength, Math.toRadians(270)))
                .lineToY(-61,null, new ProfileAccelConstraint(-40, 40))
                ;

        TrajectoryActionBuilder drive4 = drive.actionBuilder(new Pose2d(51, -61, Math.toRadians(270)))
                .strafeTo(new Vector2d(42, -50), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(-8 + halfWidth, -45.5 + halfLength, Math.toRadians(83)), Math.toRadians(83), null, new ProfileAccelConstraint(-80, 80))
                ;

        TrajectoryActionBuilder drive5 = drive.actionBuilder(new Pose2d(-15 + halfWidth, -45 + halfLength, Math.toRadians(90)))
                .lineToY(-50 + halfLength, null, new ProfileAccelConstraint(-90, 90))
                .strafeTo(new Vector2d(47 - halfWidth, -45.5 + halfLength), null, new ProfileAccelConstraint(-90, 90))
                .setTangent(Math.toRadians(90))
                .lineToY(-22 + halfLength, null, new ProfileAccelConstraint(-90, 90))
                .strafeTo(new Vector2d(53, -22 + halfLength),null, new ProfileAccelConstraint(-90, 90))
                .setTangent(Math.toRadians(90))
                .lineToY(-52, null, new ProfileAccelConstraint(-80, 80))
                /*.lineToY(-22 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(63, -22 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-52) */
                .strafeTo(new Vector2d(51,-35), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(51, -54 + halfLength, Math.toRadians(245)),Math.toRadians(245), null, new ProfileAccelConstraint(-40, 40))
                ;

        TrajectoryActionBuilder drive6 = drive.actionBuilder(new Pose2d(51, -54 + halfLength, Math.toRadians(270)))
                .lineToY(-61,null, new ProfileAccelConstraint(-40, 40))
                ;
        TrajectoryActionBuilder drive7 = drive.actionBuilder(new Pose2d(51, -61, Math.toRadians(270)))
                .strafeTo(new Vector2d(30, -46), null, new ProfileAccelConstraint(-80, 80))
                ;

        TrajectoryActionBuilder drive7half = drive.actionBuilder(new Pose2d(30, -46, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(halfWidth + 2, -45.7 + halfLength, Math.toRadians(78)), Math.toRadians(78), null, new ProfileAccelConstraint(-60, 60))
        ;

        TrajectoryActionBuilder driveBack = drive.actionBuilder(new Pose2d(halfWidth + 2, -45.7 + halfLength, Math.toRadians(90)))
                .lineToY(-47 + halfLength)
                ;

        TrajectoryActionBuilder drive8 = drive.actionBuilder(new Pose2d(halfWidth + 2, -44.5 + halfLength,Math.toRadians(90)))
                .lineToY(-50 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(47 - halfWidth, -45.5 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(51, -54 + halfLength, Math.toRadians(245)),Math.toRadians(245), null, new ProfileAccelConstraint(-40, 40))
                ;
        TrajectoryActionBuilder drive9 = drive.actionBuilder(new Pose2d(51, -54 + halfLength, Math.toRadians(270)))
                .lineToY(-60,null, new ProfileAccelConstraint(-40, 40))
                ;

        /*TrajectoryActionBuilder drive1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-12 + halfWidth, -39.5 + halfLength))
                .waitSeconds(1)
                .lineToY(-42.5 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(47 - halfWidth, -45.5 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-22 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(53, -22 + halfLength),null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-52, null, new ProfileAccelConstraint(-80, 80))
                .lineToY(-22 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(63, -22 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-52)
                .lineToY(-22 + halfLength)
                .waitSeconds(0.01)
                .strafeTo(new Vector2d(71.5, -22 + halfLength))
                .setTangent(Math.toRadians(90))
                .lineToY(-52)
                .strafeTo(new Vector2d(64,-42), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(52, -76 + halfLength, Math.toRadians(250)),Math.toRadians(250), null, new ProfileAccelConstraint(-80, 80))
                .waitSeconds(1)
                .strafeTo(new Vector2d(42, -65 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(-6, -39.5 + halfLength, Math.toRadians(90)),Math.toRadians(90), null, new ProfileAccelConstraint(-80, 80))
                .waitSeconds(1)
                .strafeTo(new Vector2d(5, -45 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(45, -76 + halfLength, Math.toRadians(250)),Math.toRadians(250), null, new ProfileAccelConstraint(-80, 80))
                .waitSeconds(1)
                .strafeTo(new Vector2d(35, -65 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(-6, -39.5 + halfLength, Math.toRadians(90)),Math.toRadians(90), null, new ProfileAccelConstraint(-80, 80));
                */


        Action trajectoryAction1 = drive1.build();
        Action trajectoryAction2 = drive2.build();
        Action trajectoryAction3 = drive3.build();
        Action sleep = new SleepAction(0.3);
        Action sleep2 = new SleepAction(0.3);
        Action sleep3 = new SleepAction(0.3);
        Action trajectoryAction4 = drive4.build();
        Action trajectoryAction5 = drive5.build();
        Action trajectoryAction6 = drive6.build();
        Action trajectoryAction7 = drive7.build();
        Action trajectoryAction7half = drive7half.build();
        Action trajectoryAction8 = drive8.build();
        Action trajectoryAction9 = drive9.build();
        Action trajectoryActionBack = driveBack.build();

        Actions.runBlocking(closeTopClaw);

        while (!isStarted() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Autonomous", "Started");

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction (
                                liftToHighJunction,
                                trajectoryAction1,
                                flipTClawOut,
                                rotateTClaw),
                            new SequentialAction (
                                    liftToLowPosition,
                                    openTopClaw,
                                    flipTClawOut3),
                            new ParallelAction (
                                    rotateTClaw2,
                                    liftDown,
                                    trajectoryAction5),
                            /*new ParallelAction (
                                    flipTClawOut2,
                                    openTopClaw2,
                                    rotateTClaw2,
                                    trajectoryAction3),*/
                            new SequentialAction (
                                    flipTClawOut5,
                                    trajectoryAction3,
                                    closeTopClaw2,
                                    sleep),
                            new ParallelAction(
                                    liftToHighJunction2,
                                    trajectoryAction4),
                            new SequentialAction(
                                    liftToLowPosition2,
                                    openTopClaw3),
                            new ParallelAction(
                                    liftDown2,
                                    trajectoryAction2,
                                    flipTClawOut4,
                                    openTopClaw4,
                                    rotateTClaw3),
                            new SequentialAction(
                                    trajectoryAction6,
                                    closeTopClaw3,
                                    sleep2
                            ),
                            new ParallelAction(
                                    liftToHighJunction3,
                                    trajectoryAction7
                            ),
                            new SequentialAction(
                                    trajectoryAction7half,
                                    liftToLowPosition3,
                                    openTopClaw5,
                                    trajectoryActionBack,
                                    liftDown3)
                            /*new ParallelAction(
                                    trajectoryAction8,
                                    flipTClawOut5,
                                    openTopClaw6
                            ),
                            new SequentialAction(
                                    trajectoryAction9,
                                    closeTopClaw4,
                                    sleep3
                            ) */
                            )
                    /*new SequentialAction (
                            closeTopClaw,
                            liftToHighJunction,
                            flipTClawOut,
                            rotateTClaw,
                            trajectoryAction1,
                            liftToLowPosition,
                            openTopClaw,
                            flipTClawOut3,
                            liftDown,
                            trajectoryAction2,
                            flipTClawOut2,
                            openTopClaw2,
                            rotateTClaw2,
                            trajectoryAction3,
                            closeTopClaw2,
                            sleep,
                            liftToHighJunction2,
                            trajectoryAction4,
                            liftToLowPosition2,
                            openTopClaw3,
                            liftDown2,
                            trajectoryAction5,
                            flipTClawOut4,
                            openTopClaw4,
                            rotateTClaw3,
                            closeTopClaw3,
                            sleep2,
                            trajectoryAction6,
                            slideOut
                    )*/
            );
            telemetry.addData("Autonomous", "Complete");
            telemetry.update();
        }
    }
}