package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

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
import com.sun.tools.javac.util.MandatoryWarningHandler;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.unit.UnitConverter;
import org.firstinspires.ftc.teamcode.tatooine.utils.unit.unit;
import org.opencv.core.Mat;

@Autonomous(name = "1+3", group = "Autonomous")
public class CloseToBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Action trajectoryActionChosen;
        Pose2d beginPose = new Pose2d(6+(18.0/2.0) - 30, -(72-(18.0/2.0)), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, false, false);
        wrist.setPosAng(0.5);
        Arm arm = new Arm(this, false);
        arm.setStartAngle(41+5);
        TrajectoryActionBuilder trajectoryRed = drive.actionBuilder(beginPose)
                //start Wrist
                .stopAndAdd(new SequentialAction(new InstantAction(wrist::stright),new InstantAction(wrist::stright),new InstantAction(wrist::stright)))
                //get arm prep for score specimen
                .afterTime(0, new ParallelAction(new InstantAction(wrist::stright),arm.setAngle(46.5), arm.setExtension(19)))
                //go score specimen
                .splineToLinearHeading(new Pose2d(-10, -32, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                //close extention
                .stopAndAdd(arm.setExtension(0))
                //go back
                .strafeTo(new Vector2d(-10,-40))
                //go for intake pos
                .splineToLinearHeading(new Pose2d(-55, -36 , Math.toRadians(90)), Math.toRadians(90))
//                arm intake
                .stopAndAdd(new SequentialAction(new InstantAction (wrist::openMin),new SleepAction(0.5), intake.intake(), arm.setAngle(7),new SleepAction(0.5), intake.setPowerAction(0), arm.setAngle(30)))
                .strafeTo(new Vector2d(-50, -50))
                //go for score sample
                .splineToLinearHeading(new Pose2d(-63,-61, Math.toRadians(45)), Math.toRadians(-90))
//                //score sample
                .stopAndAdd(new SequentialAction(new ParallelAction(arm.setAngle(90),  arm.setExtension(arm.getMaxExtend())), new SleepAction(2), new InstantAction(wrist::scoreSample), new SleepAction(0.5),intake.outtake(), new SleepAction(0.5), intake.setPowerAction(0)))
                //go arm down
                .stopAndAdd(new SequentialAction(new InstantAction(wrist::openMin), new SleepAction(0.5), arm.setExtension(0),new SleepAction(1),  arm.setAngle(35)))
//                //go for intake pos1.5
                .turnTo(Math.toRadians(102.5))
                .stopAndAdd(new SequentialAction(arm.setAngle(22), arm.setExtension(46), new SleepAction(1.6), new InstantAction(()-> arm.setPowerExtend(0)), new InstantAction (wrist::openMin),new SleepAction(0.5),  intake.intake(),arm.setAngle(15), arm.setAngle(7),new SleepAction(0.5), intake.setPowerAction(0), arm.setAngle(30),arm.setExtension(0),  new InstantAction(wrist::stright)))
                .stopAndAdd(new SequentialAction(arm.setExtension(0)))
                .turnTo(Math.toRadians(45))
//                //score sample
                .stopAndAdd(new SequentialAction(new ParallelAction(arm.setAngle(90),  arm.setExtension(arm.getMaxExtend())), new SleepAction(1.5), new InstantAction(wrist::scoreSample), new SleepAction(0.5),intake.outtake(), new SleepAction(0.5), intake.setPowerAction(0)))
                //go arm down
                .stopAndAdd(new SequentialAction(new InstantAction(wrist::openMin), new SleepAction(0.5), arm.setExtension(0),new SleepAction(1),  arm.setAngle(35)))
                .strafeTo(new Vector2d(-40, 0))
                .splineTo(new Vector2d(-20, 0), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(arm.setAngle(60), arm.setExtension(10), new InstantAction(()-> arm.setPowerExtend(0)),  new InstantAction(wrist::stright)))
                ;


        //.splineTo(new Vector2d(-36.70, -63.40), Math.toRadians(202.26));


                trajectoryActionChosen = trajectoryRed.build();

        while (opModeInInit() && !isStopRequested()){
            arm.setPowerAngleWithF(0);
            wrist.close();
            telemetry.addData("getAngle", arm.getAngle());
            telemetry.update();
        }
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(wrist::stright),
                        new ParallelAction(arm.moveAngle(),
                        trajectoryActionChosen
                )
                ));


    }
}