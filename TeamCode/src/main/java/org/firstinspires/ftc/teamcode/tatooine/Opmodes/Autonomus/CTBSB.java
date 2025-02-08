package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;

@Autonomous(name = "1+3", group = "Autonomous")
public class CTBSB extends LinearOpMode {


    @Override
    public void runOpMode(){
        Action trajectoryActionChosen;
        Pose2d beginPose = new Pose2d(6+(18.0/2.0) - 30, -(72-(18.0/2.0)), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, false);
        Arm arm = new Arm(this, false);
        TrajectoryActionBuilder goScoreSpecimen = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-10, -32, Math.toRadians(90)), Math.toRadians(90));

    }
}
