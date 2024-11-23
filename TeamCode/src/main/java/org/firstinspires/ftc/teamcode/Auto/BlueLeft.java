package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        // initialize actions

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

        double scorexPos = 10.5;
        double scoreyPos = 18.0;
        double pickUpxPos1 = 18;
//        double pickUpxPos2 = 21;

        //driveToScorePre
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(scorexPos, 0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.25);
        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.25);

        //driveToFirstPickUp
        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .waitSeconds(.25)
                .strafeToLinearHeading(new Vector2d(pickUpxPos1, 12), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.25);

        //driveToScoreFirst
        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint)
                .waitSeconds(.25);

        //parking
        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(25, -40), Math.toRadians(-90),
                        baseVelConstraint,
                        baseAccelConstraint)
//                .splineToLinearHeading(new Pose2d(10, -80, Math.toRadians(0)), Math.PI / 2)
                .waitSeconds(.25);


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
        Action driveToScoreFirst = traj4.build();
        Action park = traj5.build();



        // Start

        telemetry.addData("Status", "Running");
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        // Preload
                        driveToScorePre1,
                        driveToScorePre2
                )
        );

        viperSlide.goToPosition(4250);
        viperSlide.bucketScore();
        sleep(1250);
        viperSlide.bucketRest();
        viperSlide.goToRest();

        hSlide.resetEncoder();
        viperSlide.resetEncoders();

        // First sample
        Actions.runBlocking(driveToFirstPickUp1);
        hSlide.goToPosition(80);
        intake.wristDown();
        sleep(300);
        hSlide.goToPosition(150);

        intake.grabberSuck();
        sleep(2500);
        intake.grabberOff();

        intake.wristUp();
        hSlide.goToRest();
        intake.grabberSpit();
        sleep(1500);
        intake.grabberOff();

        Actions.runBlocking(driveToScoreFirst);

        viperSlide.goToPosition(4250);
        viperSlide.bucketScore();
        sleep(1250);
        viperSlide.bucketRest();
        viperSlide.goToRest();

        hSlide.resetEncoder();
        viperSlide.resetEncoders();

        Actions.runBlocking(park);



//        if (opModeIsActive()) {
//
//        }

    }

}
