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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.States.Conts;

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
                .strafeTo(new Vector2d(-7, -40))
                .splineToLinearHeading(new Pose2d(-27, -58, Math.toRadians(131.5)), Math.toRadians(180))
                .strafeTo(new Vector2d(-27 - 0.5, -58 + 0.5));
        TrajectoryActionBuilder goScoreSample1 = goIntakeSample1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-48, -64.5, Math.toRadians(192.5)), Math.toRadians(180));
        TrajectoryActionBuilder goIntakeSample2 = goScoreSample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-42 ,-56), Math.toRadians(112));
        TrajectoryActionBuilder goScoreSample2 = goIntakeSample2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-48, -64.5, Math.toRadians(193.5)), Math.toRadians(180));
        waitForStart();
        Actions.runBlocking(new ParallelAction(
                arm.moveExtend(), arm.moveAngle(), new SequentialAction(
                new InstantAction(wrist::intakeFlat),
                new ParallelAction(
                        new InstantAction(wrist::intakeFlat),
                        intake.intakeByTimer(1, 0.1)
                        , arm.setAngle(Conts.angleScoreSpecimenHigh)
                        , arm.setExtend(15)
                        , goScoreSpecimen.build())
                , arm.setAngle(45)
                , arm.setExtend(0)
                , new ParallelAction(goIntakeSample1.build(), new SequentialAction(intake.intakeByTimer(-1, 0.5), intake.intakeByTimer(1, 0.5)))
                , arm.setAngle(5)
                , arm.setExtend(20)
                , new ParallelAction(arm.setAngle(-1), arm.setExtend(arm.getMaxExtend() - 4))
                , new ParallelAction(goScoreSample1.build(), arm.setAngle(63))
                , intake.intakeByTimer(-1, 0.5)
                , intake.intakeByTimer(1, 0.5)
                , goIntakeSample2.build()
                , new ParallelAction(arm.setExtend(arm.getMaxExtend() - 27.5)  , new SequentialAction(
                arm.setAngle(30)
                , arm.setAngle(-1)
                ,arm.setExtend(arm.getMaxExtend() - 18)
                ))
                , new SleepAction(1)

                , new ParallelAction(goScoreSample2.build(), arm.setAngle(65), arm.setExtend(arm.getMaxExtend()- 4))
                , intake.intakeByTimer(-1, 0.5)
                , intake.intakeByTimer(1, 0.5)
        )
        ));
    }

}