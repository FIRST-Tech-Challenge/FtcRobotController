package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.BackupAutos;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedTerminalAuto extends PowerPlay_AprilTagDetectionDeposit {
    private Slide slideControl;
    private Arm armControl;
    private Claw clawControl;
    boolean isAuto;

    //initialize the needed objects and variables
    public void initialize(){
        slideControl = new Slide(hardwareMap);
        armControl = new Arm(hardwareMap);
        clawControl = new Claw(hardwareMap);

        isAuto = true;
        clawControl.toggleWristRotate();
        clawControl.toggleOpenClose();
    }

    //tag detection method
    public void detectTag(){
        super.runOpMode();
    }

    public void runOpMode(){
        initialize();
        Pose2d startPose = new Pose2d(0,0,0);
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose); //Technically useless but ill keep it in just in case

        TrajectorySequence beginingMove = bot.trajectorySequenceBuilder(startPose)
                .forward(5)
                .turn(180)
                .build();

        TrajectorySequence goToHighJunction = bot.trajectorySequenceBuilder(beginingMove.end())
                .addTemporalMarker(2,()->{
                    slideControl.setHighJunction(telemetry);
                    armControl.setExtake();
                    clawControl.toggleWristRotate();
                    clawControl.wristInExtakePosition = false;
                    clawControl.toggleOpenClose();
                    clawControl.toggleOpenClose();
                    clawControl.toggleWristRotate();
                    slideControl.setIntakeOrGround();
                    armControl.setIntake();
                })
                .lineToLinearHeading(new Pose2d(10,52,23))
                .build();

        TrajectorySequence zoneTwo = bot.trajectorySequenceBuilder(goToHighJunction.end())
                .lineToLinearHeading(new Pose2d(0,24,0))
                .build();

        TrajectorySequence zoneThree = bot.trajectorySequenceBuilder(goToHighJunction.end())
                .lineToLinearHeading(new Pose2d(0,24,0))
                .strafeRight(24)
                .build();

        TrajectorySequence zoneOne = bot.trajectorySequenceBuilder(goToHighJunction.end())
                .lineToLinearHeading(new Pose2d(0,24,0))
                .strafeLeft(24)
                .build();

        waitForStart();
        bot.followTrajectorySequence(beginingMove);
        detectTag();
        bot.followTrajectorySequence(goToHighJunction);

        if(tagUse == 1)
            bot.followTrajectorySequence(zoneOne);
        else if(tagUse == 2)
            bot.followTrajectorySequence(zoneTwo);
        else
            bot.followTrajectorySequence(zoneThree);

        while(opModeIsActive()){
            bot.update();
            armControl.update(telemetry);
            slideControl.update(telemetry);
        }
    }



}
