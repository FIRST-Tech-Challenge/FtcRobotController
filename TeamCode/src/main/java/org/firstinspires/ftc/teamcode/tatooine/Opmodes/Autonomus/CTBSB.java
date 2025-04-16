package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import androidx.annotation.NonNull;
import androidx.appcompat.widget.SearchView;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.States.Conts;
@Disabled
@Autonomous(name = "1+3", group = "Autonomous")
public class CTBSB extends LinearOpMode {


    @Override
    public void runOpMode() {
        Action trajectoryActionChosen;
        Pose2d beginPose = new Pose2d(6 + (18.0 / 2.0) - 30, -(72 - (18.0 / 2.0)), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, false);
        Arm arm = new Arm(this, false);
        TrajectoryActionBuilder goScoreSpecimen = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-7, -31.5, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder goIntakeSample1 = goScoreSpecimen.endTrajectory().fresh()
                .strafeTo(new Vector2d(-8, -38))
                .splineToLinearHeading(new Pose2d(-28, -58 + 1, Math.toRadians(129)), Math.toRadians(180));
        TrajectoryActionBuilder goScoreSample1 = goIntakeSample1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-52, -55, Math.toRadians(207)), Math.toRadians(180));
        TrajectoryActionBuilder goIntakeSample2 = goScoreSample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-43.5  ,-56 + 0.5), Math.toRadians(115));
        TrajectoryActionBuilder goScoreSample2 = goIntakeSample2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-48, -61, Math.toRadians(205)), Math.toRadians(180));
        TrajectoryActionBuilder goIntakeSample3 = goScoreSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-42 ,-56), Math.toRadians(120))
                .strafeToLinearHeading(new Vector2d(-42+3 , -56-3), Math.toRadians(132.5))
                ;
        TrajectoryActionBuilder goIntakeSample3Half = goIntakeSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-42+3-5 , -56-3+ 5), Math.toRadians(132.5))
                ;
        TrajectoryActionBuilder goScoreSample3 = goIntakeSample3Half.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)), Math.toRadians(-90))
                ;

        waitForStart();
        Actions.runBlocking(new ParallelAction(
                arm.moveExtend(), arm.moveAngle(), new SequentialAction(
                new InstantAction(wrist::intakeFlat),
                new ParallelAction(
                        new InstantAction(wrist::intakeFlat),
                        intake.intakeByTimer(1, 0.1)
                        , arm.setAngle(Conts.angleScoreSpecimenHigh)
                        , arm.setExtend(14.3)
                        , goScoreSpecimen.build())
                , new SequentialAction(
                        new ParallelAction(
                new InstantAction(()-> arm.setAngle(46)))
                , arm.setExtend(0)
                )
                , new ParallelAction(goIntakeSample1.build(), new SequentialAction(intake.intakeByTimer(-1, 0.3), intake.intakeByTimer(1, 0.01)))
                , arm.setAngle(5)
                , arm.setExtend(20)
                , new ParallelAction(arm.setAngle(-1), arm.setExtend(arm.getMaxExtend() - 4))
                , new ParallelAction(goScoreSample1.build(), arm.setAngle(70))
                , intake.intakeByTimer(-0.25, 0.5)
                , arm.setAngle(66)
                , intake.intakeByTimer(1, 0.01)
                , goIntakeSample2.build()
                , new ParallelAction(arm.setExtend(arm.getMaxExtend() - 30)  , new SequentialAction(
                        arm.setAngle(30)))
                ,arm.setAngle(-1)
                ,arm.setExtend(arm.getMaxExtend() - 12.0)
                , new SleepAction(1)
                , arm.setExtend(arm.getMaxExtend() - 10.0)
                ,  new SleepAction(1)
                , new InstantAction(wrist::intakeFlat)
                , new InstantAction(wrist::intakeFlat)
                , arm.setAngle(40)
                , new ParallelAction( new InstantAction(wrist::intakeFlat), goScoreSample2.build(), arm.setAngle(70),  new InstantAction(wrist::intakeFlat), arm.setExtend(arm.getMaxExtend()-4 ))
                , intake.intakeByTimer(-0.25, 0.5)
                , intake.intakeByTimer(1, 0.01)
                , goIntakeSample3.build()
                , new InstantAction(wrist::intakeFlat)
                , arm.setAngle(20)
                , arm.setAngle(0)
                , goIntakeSample3Half.build()
                , new SleepAction(1)
                ,new ParallelAction(
                arm.setAngle(20)
                , goScoreSample3.build()
                )
                , new ParallelAction(arm.setAngle(Conts.angleScoreSample), arm.setExtend(arm.getMaxExtend() - 0.5))
                , new InstantAction(wrist::doc),new SleepAction(0.5), intake.intakeByTimer(-0.25, 0.5)
                , new SequentialAction(intake.stop(),new InstantAction(wrist::intakeFlat), arm.setExtend(0), arm.setAngle(Conts.angleDrive))
        )
        ));
    }

}