package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.ViperSlide;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketFlapAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketRestAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketScoreAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSuckAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperDownForTimeAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperStopAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperToPositionAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.hSlide.hSlideToPositionAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperToRestAction;

@Autonomous(name="AutoTest", group="Test")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ViperSlide viperSlide = new ViperSlide(this);
        HorizontalSlide hSlide = new HorizontalSlide(this, 3);
        Intake intake = new Intake(this, hSlide);

        ViperToPositionAction viperUp = new ViperToPositionAction(viperSlide, 3000);
        ViperToPositionAction viperAllTheWayUpAtTheTopToScoreASampleInHighBucket = new ViperToPositionAction(viperSlide, 4250);
        ViperToPositionAction viperDown = new ViperToPositionAction(viperSlide, 0);
        ViperToRestAction viperToRest = new ViperToRestAction(viperSlide);
        ViperDownForTimeAction viperDownForTime = new ViperDownForTimeAction(viperSlide, 1000);
        ViperStopAction viperStop = new ViperStopAction(viperSlide);

        hSlideToPositionAction hSlideForward = new hSlideToPositionAction(hSlide, 500);
        hSlideToPositionAction hSlideBackward = new hSlideToPositionAction(hSlide, 0);

        GrabberSuckAction grabberSuck = new GrabberSuckAction(intake, 1000);

        BucketFlapAction openBucket = new BucketFlapAction(viperSlide, "open", 1000);
        BucketFlapAction closeBucket = new BucketFlapAction(viperSlide, "close");
        BucketScoreAction bucketScore = new BucketScoreAction(viperSlide);
        BucketRestAction bucketRest = new BucketRestAction(viperSlide);

        SleepAction sleep = new SleepAction(1);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Initialized", "");
            telemetry.update();

            System.out.println("ViperSlide pos: " + viperSlide.getPos());
        }

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        viperUp
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        closeBucket,
                        bucketScore
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        viperAllTheWayUpAtTheTopToScoreASampleInHighBucket,
                        viperStop,
                        openBucket
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        viperDownForTime
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        bucketRest
                )
        );



        Actions.runBlocking(
                viperToRest
        );

//        Actions.runBlocking(
//                new ParallelAction(
////                        hSlideBackward,
//                        viperToRest
//                )
//        );

        if (isStopRequested() || gamepad1.b) return;
    }
}
