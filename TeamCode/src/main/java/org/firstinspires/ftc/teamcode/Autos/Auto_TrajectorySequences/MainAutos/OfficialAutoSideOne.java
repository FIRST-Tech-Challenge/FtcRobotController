package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Quad Four And Two")
public class OfficialAutoSideOne extends PowerPlay_AprilTagDetectionDeposit {
    Arm armControl;
    Slide slideControl;
    Claw clawControl;

    public void initialize(){
        //initializing needed objects
        armControl = new Arm(hardwareMap);
        slideControl = new Slide(hardwareMap);
        clawControl = new Claw(hardwareMap);

    }

    @Override
    public void runOpMode() {
        initialize();

        Pose2d startPose = new Pose2d(61.8,-35,Math.toRadians(180));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        //trajectory sequences
        TrajectorySequence strafeRightandForward = bot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(61.8+22,-35))
                .forward(30)
                .build();

        
    }
}