package org.firstinspires.ftc.teamcode.Auto;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.MainDrive;
import org.firstinspires.ftc.teamcode.Components.ViperSlide;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

import java.util.Arrays;


@Autonomous(name="BlueLeft", group="Auto")
public class BlueLeft extends LinearOpMode {
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

        double scorexPos = 10.5;
        double scoreyPos = 18.0;
        double pickUpxPos1 = 18;
        double pickUpxPos2 = 20;

        //driveToScorePre
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(scorexPos, 0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.5);
        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.5);

        //driveToFirstPickUp
        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos1, 12), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.5);
        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos2, 12), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.5);

        //driveToScoreFirst
        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.5);

        //driveToSecondPickUp
        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos1, 23), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.5);
        TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(pickUpxPos2, 23), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.5);

        //driveToScoreSecond
        TrajectoryActionBuilder traj8 = traj7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.5);

        // Between initialization and start
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Action driveToScorePre1 = traj1.build();
        Action driveToScorePre2 = traj2.build();



        Action driveToFirstPickUp1 = traj3.build();
        Action driveToFirstPickUp2 = traj4.build();
        Action driveToScoreFirst = traj5.build();
        Action driveToSecondPickUp1 = traj6.build();
        Action driveToSecondPickUp2 = traj7.build();
        Action driveToScoreSecond = traj8.build();


        // Start

        telemetry.addData("Status", "Running");
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        // Preload
                        driveToScorePre1,
                        driveToScorePre2,
                        // TODO raise viper, flip bucketx2, lower vipers

                        // First sample
                        driveToFirstPickUp1,
                        // TODO extend hSlide, flip wrist
                        driveToFirstPickUp2,
                        // TODO retract all pickup stuff and put in bucket
                        driveToScoreFirst,
                        // TODO *same

                        // Second sample
                        driveToSecondPickUp1,
                        // TODO *same
                        driveToSecondPickUp2,
                        // TODO *same
                        driveToScoreSecond
                        // TODO *same

                )
        );

//        if (opModeIsActive()) {
//
//        }



    }

}
// viper max limit4300