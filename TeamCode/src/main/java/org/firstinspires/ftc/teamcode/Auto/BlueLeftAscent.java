package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketFlapAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketRestAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketScoreAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSpitAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSuckAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.hSlide.hSlideToPositionAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperDownForTimeAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperStopAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperToPositionAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperToRestAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.wrist.WristDownAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.wrist.WristUpAction;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.MainDrive;
import org.firstinspires.ftc.teamcode.Components.ViperSlide;

// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

import java.util.Arrays;

@Autonomous(name="BlueLeftAscent", group="Auto")
public class BlueLeftAscent extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // todo when we add huskyLens
        // Initializing huskyLens
        int visionOutputPosition = 1;

        // Initializing our classes
        HorizontalSlide hSlide = new HorizontalSlide(this, 3);
        ViperSlide viperSlide = new ViperSlide(this);
        Intake intake = new Intake(this, hSlide);
        MainDrive mainDrive = new MainDrive(this);

        hSlide.resetEncoder();
        viperSlide.resetEncoders();

        // RR-specific initialization
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Set velocity and accel constraints
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-25.0, 40);

        VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint slowAccelConstraint = new ProfileAccelConstraint(-10, 20);

        double scorexPos = 11.2;
        double scoreyPos = 17.3;
        double pickUpxPos1 = 16;
        double pickUpxPos2 = 22;

        //driveToScorePre
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(scorexPos, 0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.25);
        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        slowVelConstraint,
                        slowAccelConstraint);

        //driveToFirstPickUp
        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos1, 12), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);

        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos2, 12), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToScoreFirst
        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint)
//                .splineToLinearHeading(new Pose2d(10, -80, Math.toRadians(0)), Math.PI / 2)
                ;

        //driveToSecondPickUp
        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos1, 22), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);
        TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos2, 22), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToScoreSecond
        TrajectoryActionBuilder traj8 = traj7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToThirdPickUp
        TrajectoryActionBuilder traj9 = traj8.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos1, 27), Math.toRadians(45),
                        baseVelConstraint,
                        baseAccelConstraint);

        //Actions
        ViperToPositionAction viperMid = new ViperToPositionAction(viperSlide, 2500);
        ViperToPositionAction viperAllTheWayUpAtTheTopToScoreASampleInHighBucket = new ViperToPositionAction(viperSlide, 4350);
        ViperDownForTimeAction viperDownForTime = new ViperDownForTimeAction(viperSlide, 1000);
        ViperToRestAction viperToRest = new ViperToRestAction(viperSlide);
        ViperStopAction viperStop = new ViperStopAction(viperSlide);

        BucketFlapAction bucketOpen = new BucketFlapAction(viperSlide, "open", 800);
        BucketFlapAction bucketClose = new BucketFlapAction(viperSlide, "close");
        BucketScoreAction bucketScore = new BucketScoreAction(viperSlide);
        BucketRestAction bucketRest = new BucketRestAction(viperSlide);

        hSlideToPositionAction hSlideForward = new hSlideToPositionAction(hSlide, 80);
        hSlideToPositionAction hSlideBackward = new hSlideToPositionAction(hSlide, -10);

        WristDownAction wristDown = new WristDownAction(intake);
        WristUpAction wristUp = new WristUpAction(intake);

        GrabberSuckAction grabberSuck = new GrabberSuckAction(intake, 2000);
        GrabberSpitAction grabberSpit = new GrabberSpitAction(intake, 2000, 750);


        // Between initialization and start
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested() || gamepad1.b) return;

        Action driveToScorePre1 = traj1.build();
        Action driveToScorePre2 = traj2.build();
        Action driveToFirstPickUp1 = traj3.build();
        Action driveToFirstPickUp2 = traj4.build();
        Action driveToScoreFirst = traj5.build();
        Action driveToSecondPickUp1 = traj6.build();
        Action driveToSecondPickUp2 = traj7.build();
        Action driveToScoreSecond = traj8.build();
        Action driveToThirdPickUp1 = traj9.build();


        // Start
        telemetry.addData("Status", "Running");
        telemetry.update();

        // score pre
        Actions.runBlocking(driveToScorePre1);
        Actions.runBlocking(
                new ParallelAction(
                        driveToScorePre2,
                        viperMid
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        bucketClose,
                        bucketScore
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        viperAllTheWayUpAtTheTopToScoreASampleInHighBucket,
                        viperStop,
                        bucketOpen
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        viperDownForTime,
                        hSlideForward
                )
        );

        // pick up first
        Actions.runBlocking(
                new ParallelAction(
                        driveToFirstPickUp1,
                        viperToRest,
                        bucketRest,
                        wristDown
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        driveToFirstPickUp2,
                        grabberSuck
                )
        );

        viperSlide.resetEncoders();

        // drive score first
        Actions.runBlocking(
                new ParallelAction(
                        driveToScoreFirst,
                        hSlideBackward,
                        wristUp,
                        grabberSpit
                )
        );

        hSlide.resetEncoder();

        // score first
        Actions.runBlocking(viperMid);
        Actions.runBlocking(
                new ParallelAction(
                        bucketClose,
                        bucketScore
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        viperAllTheWayUpAtTheTopToScoreASampleInHighBucket,
                        viperStop,
                        bucketOpen
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        viperDownForTime,
                        hSlideForward
                )
        );

        //pick up second
        Actions.runBlocking(
                new ParallelAction(
                        driveToSecondPickUp1,
                        viperToRest,
                        bucketRest,
                        wristDown
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        driveToSecondPickUp2,
                        grabberSuck
                )
        );

        viperSlide.resetEncoders();

        // drive score second
        Actions.runBlocking(
                new ParallelAction(
                        driveToScoreSecond,
                        hSlideBackward,
                        wristUp,
                        grabberSpit
                )
        );

        hSlide.resetEncoder();

        // score second
        Actions.runBlocking(viperMid);
        Actions.runBlocking(
                new ParallelAction(
                        bucketClose,
                        bucketScore
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        viperAllTheWayUpAtTheTopToScoreASampleInHighBucket,
                        viperStop,
                        bucketOpen
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        viperDownForTime
//                        hSlideForward
                )
        );

        //pick up third
        Actions.runBlocking(
                new ParallelAction(
//                        driveToThirdPickUp1,
                        viperToRest,
                        bucketRest
//                        wristDown
                )
        );
//        Actions.runBlocking(
//                new ParallelAction(
//                        driveToSecondPickUp2,
//                        grabberSuck
//                )
//        );

        viperSlide.resetEncoders();


//        if (opModeIsActive()) {
//
//        }



    }

}
// viper max limit4300