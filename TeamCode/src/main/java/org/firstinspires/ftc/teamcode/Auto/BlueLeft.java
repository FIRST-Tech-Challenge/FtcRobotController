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


@Autonomous(name="BlueLeft", group="Auto")
public class BlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // todo when we add huskyLens
        // Initializing huskyLens
        int visionOutputPosition = 5;

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

//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(4, 0))
//                .waitSeconds(2);
//
//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(2, -2), Math.toRadians(45))
//                .build();


        // Between initialization and start
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(initialPose).strafeTo(new Vector2d(6,6)).waitSeconds(2).build());

//        Action trajectoryActionChosen = tab1.build();

        // Start

        telemetry.addData("Status", "Running");
        telemetry.update();

//        Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen,
//                        trajectoryActionCloseOut
//                )
//        );

//        if (opModeIsActive()) {
//
//        }



    }

}
// viper max limit4300