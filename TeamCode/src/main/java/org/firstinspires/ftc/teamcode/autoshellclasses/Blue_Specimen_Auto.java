package org.firstinspires.ftc.teamcode.autoshellclasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.ViperArmActions;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.WristClawActions;
import org.firstinspires.ftc.teamcode.PinpointDrive;


@Config
@Autonomous(name = "Blue_Specimen_Auto", group = "Autonomous")
public class Blue_Specimen_Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Initialization steps
        //Creates instance of MechanismActionBuilders
        WristClawActions _WristClawActions = new WristClawActions(this);
        ViperArmActions _ViperArmActions = new ViperArmActions(this);

        //Initializes Pinpoint
        Pose2d initialPose = new Pose2d(-12, 62, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //closes claw on init
        Actions.runBlocking(_WristClawActions.CloseClaw());

        telemetry.update();
        waitForStart();
        //----------------------------------------------------------------------------------------------

        if (isStopRequested()) return;

        Vector2d clip_position1 = new Vector2d(0, 36);
        Vector2d clip_position2 = new Vector2d(5, 36);
        Vector2d clip_position3 = new Vector2d(10, 36);
        double clip_heading = Math.toRadians(-90);

        Vector2d sample_drop_position1 = new Vector2d(-46,55);
        Vector2d sample_drop_position2 = new Vector2d(-56,55);
        Vector2d sample_drop_position3 = new Vector2d(-60,55);

        Vector2d outer_sample_position = new Vector2d(-46, 15);
        Vector2d middle_sample_position = new Vector2d(-56, 15);
        Vector2d inner_sample_position = new Vector2d(-60, 15);
        double sample_heading = Math.toRadians(-90);

        double rotation_speed = Math.toRadians(0.5);

        Action driveToClip1, driveToClip2, driveToClip3, sampleDropOff1, sampleDropOff2, sampleDropOff3, samplePickup1, samplePickup2, samplePickup3, driveToPark, armUpWait1, armUpWait2, armUpWait3, armDownWait1, armDownWait2, armDownWait3, viperUpWait1, viperUpWait2, viperUpWait3, viperDownWait1, viperDownWait2, viperDownWait3, wristUpWait1, wristUpWait2, wristUpWait3, wristDownWait1, wristDownWait2, wristDownWait3, clawOpenWait1, clawOpenWait2, clawOpenWait3, clawCloseWait1, clawCloseWait2, clawCloseWait3;

        driveToClip1 = drive.actionBuilder(drive.pose)
                .build();
        driveToClip2 = drive.actionBuilder(new Pose2d(inner_sample_position, sample_heading))
                .build();
        driveToClip3 = drive.actionBuilder(new Pose2d(middle_sample_position, sample_heading))
                .build();

        sampleDropOff1 = drive.actionBuilder(new Pose2d(outer_sample_position, sample_heading))
                .build();
        sampleDropOff2 = drive.actionBuilder(new Pose2d(outer_sample_position, sample_heading))
                .build();
        sampleDropOff3 = drive.actionBuilder(new Pose2d(outer_sample_position, sample_heading))
                .build();

        samplePickup1 = drive.actionBuilder(new Pose2d(clip_position1, clip_heading))
                .build();
        samplePickup2 = drive.actionBuilder(new Pose2d(clip_position1, clip_heading))
                .build();
        samplePickup3 = drive.actionBuilder(new Pose2d(clip_position1, clip_heading))
                .build();

        driveToPark = drive.actionBuilder(new Pose2d(clip_position3, clip_heading))
                .build();

        armUpWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.75)
                .build();
        armUpWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.75)
                .build();
        armUpWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.75)
                .build();

        armDownWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        armDownWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        armDownWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        viperUpWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperUpWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperUpWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        viperDownWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperDownWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperDownWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        wristUpWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        wristUpWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        wristUpWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        clawCloseWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.3)
                .build();
        clawCloseWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.3)
                .build();
        clawCloseWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.3)
                .build();

        clawOpenWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.25)
                .build();
        clawOpenWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.25)
                .build();
        clawOpenWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.25)
                .build();

        Actions.runBlocking(
                new SequentialAction(

                )
        );
        while(opModeIsActive()) {
            // _leftFront.setPower(0.3);
            telemetry.update();
        }
    }
}
