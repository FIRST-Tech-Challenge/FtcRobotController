package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketFlapAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketRestAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketScoreAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketSpecimenAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSpitAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSuckAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.hSlide.hSlideToPositionAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.specimen.SpecimenGrabberGrabAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.specimen.SpecimenGrabberReleaseAction;
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

@Autonomous(name="BlueRight", group="Auto")
public class BlueRight extends LinearOpMode {
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
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Set velocity and accel constraints
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-25, 40);

        VelConstraint scoreVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint scoreAccelConstraint = new ProfileAccelConstraint(-15, 40);

        VelConstraint fastVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint fastAccelConstraint = new ProfileAccelConstraint(-30, 40);

        //Trajectories
//        double scorexPos = 11.2;
//        double scoreyPos = 17.3;
//        double pickUpxPos1 = 16;
//        double pickUpxPos2 = 22;

        //driveToScorePre
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(25.25, 0),
                        scoreVelConstraint,
                        scoreAccelConstraint);

        //driveToPushFirst
        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeTo(new Vector2d(25.25, -26),
                        baseVelConstraint,
                        baseAccelConstraint);
        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .strafeTo(new Vector2d(49, -26),
                        fastVelConstraint,
                        fastAccelConstraint);
        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeTo(new Vector2d(49, -36),
                        fastVelConstraint,
                        fastAccelConstraint);
        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .strafeTo(new Vector2d(7, -36),
                        fastVelConstraint,
                        fastAccelConstraint);

//        //driveToPushSecond
//        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(49, -36), Math.toRadians(0),
//                        fastVelConstraint,
//                        fastAccelConstraint);
//        TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
//                .strafeTo(new Vector2d(49, -46),
//                        fastVelConstraint,
//                        fastAccelConstraint);
//        TrajectoryActionBuilder traj8 = traj7.endTrajectory().fresh()
//                .strafeTo(new Vector2d(7, -46),
//                        fastVelConstraint,
//                        fastAccelConstraint);

        //spin
        TrajectoryActionBuilder spin = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15, -36), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);


        //driveToPickUpFirst
        TrajectoryActionBuilder traj9 = spin.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-1, -36), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToScoreFirst
        TrajectoryActionBuilder traj10 = traj9.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(25.25, 4), Math.toRadians(-180),
                        scoreVelConstraint,
                        scoreAccelConstraint);

        //driveToPickUpSecond
        TrajectoryActionBuilder traj11 = traj10.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(7, -36), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);
        TrajectoryActionBuilder trajafter11 = traj11.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-1, -36), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToScoreSecond
        //TODO 25.25
        TrajectoryActionBuilder traj12 = trajafter11.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(26.25, 8), Math.toRadians(-180),
                        scoreVelConstraint,
                        scoreAccelConstraint);

        //driveToPickUpSecond
        TrajectoryActionBuilder traj13 = traj12.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-1, -36), Math.toRadians(-180),
                        fastVelConstraint,
                        fastAccelConstraint);


        //Actions
        ViperToPositionAction viperSpecimen = new ViperToPositionAction(viperSlide, 2600);
        ViperDownForTimeAction viperScore = new ViperDownForTimeAction(viperSlide, 500);
        ViperToRestAction viperToRest = new ViperToRestAction(viperSlide);
        ViperStopAction viperStop = new ViperStopAction(viperSlide);

        BucketSpecimenAction bucketSpecimen = new BucketSpecimenAction(viperSlide, 1000);
        BucketRestAction bucketRest = new BucketRestAction(viperSlide);

        SpecimenGrabberGrabAction specimenGrab = new SpecimenGrabberGrabAction(viperSlide, 0);
        SpecimenGrabberReleaseAction specimenRelease = new SpecimenGrabberReleaseAction(viperSlide, 525);


        // Between initialization and start
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested() || gamepad1.b) return;

        Action driveToScorePre = traj1.build();
        Action driveToPushFirst1 = traj2.build();
        Action driveToPushFirst2 = traj3.build();
        Action driveToPushFirst3 = traj4.build();
        Action driveToPushFirst4 = traj5.build();
//        Action driveToPushSecond1 = traj6.build();
//        Action driveToPushSecond2 = traj7.build();
//        Action driveToPushSecond3 = traj8.build();
        Action spinAction = spin.build();
        Action driveToPickUpFirst = traj9.build();
        Action driveToScoreFirst = traj10.build();
        Action driveToPickUpSecond = traj11.build();
        Action driveToPickUpSecond2 = trajafter11.build();
        Action driveToScoreSecond = traj12.build();
        Action park = traj13.build();






        // Start
        telemetry.addData("Status", "Running");
        telemetry.update();

        // score pre
        Actions.runBlocking(
                new ParallelAction(
                        driveToScorePre,
                        viperSpecimen,
                        bucketSpecimen
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        viperScore,
                        specimenRelease
                )
        );

        // push first
        Actions.runBlocking(
                new ParallelAction(
                        driveToPushFirst1,
                        viperToRest,
                        bucketRest
                )
        );

        viperSlide.resetEncoders();

        Actions.runBlocking(
                new SequentialAction(
                        driveToPushFirst2,
                        driveToPushFirst3,
                        driveToPushFirst4,
//                        driveToPushSecond1,
//                        driveToPushSecond2,
//                        driveToPushSecond3,
                        spinAction,
                        driveToPickUpFirst
                )
        );

        viperSlide.grabSpecimen(); //TODO grabSpecimen action doesn't work
        sleep(500);

        // score first
        Actions.runBlocking(
                new ParallelAction(
                        driveToScoreFirst,
                        viperSpecimen,
                        bucketSpecimen
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        viperScore,
                        specimenRelease
                )
        );

        // pick up second
        Actions.runBlocking(
                new ParallelAction(
                        driveToPickUpSecond,
                        viperToRest,
                        bucketRest
                )
        );

        viperSlide.resetEncoders();

        Actions.runBlocking(driveToPickUpSecond2);

        viperSlide.grabSpecimen(); //TODO grabSpecimen action doesn't work
        sleep(500);

        // score second
        Actions.runBlocking(
                new ParallelAction(
                        driveToScoreSecond,
                        viperSpecimen,
                        bucketSpecimen
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        viperScore,
                        specimenRelease
                )
        );

        //park
        Actions.runBlocking(
                new ParallelAction(
                        park,
                        viperToRest,
                        bucketRest
                )
        );

        viperSlide.resetEncoders();



//        if (opModeIsActive()) {
//
//        }



    }

}
// viper max limit4300