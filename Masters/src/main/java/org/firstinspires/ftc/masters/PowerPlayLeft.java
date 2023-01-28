package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Config
@Autonomous(name = "Power Play Left")
public class PowerPlayLeft extends LinearOpMode {

    enum State {
        SCORE_1,
        FORWARD,
        BACK_UP_FROM_JUNCTION,
        TURN,
        SCORE_2,
        CONE_STACK1,
        PICKUP,
        BACK_UP,
        NEW_CONE_FROM_SCORE_1,
        SCORE_CONE,
        NEW_CONE,
        PARK_GRAY,
        PARK_RED,
        PARK_GREEN,
        DONE

    }

    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    int armTarget = 0, liftTarget = 0;

    int numberOfConesPickedUp = 0;

    public static double xCenterJunction = -9;
    public static double yCenterJunction = -36;

    public static double xIntermediateStack = -20;
    public static double yIntermediateStack = -12;
    public static double angleIntermediateStack = 0;
    public static double xStack = -56;
    public static double yStack = -8;

//
//    Pose2d westPoleDeposit = new Pose2d(new Vector2d(-14,-14),Math.toRadians(135));
//    Pose2d coneStack = new Pose2d(new Vector2d(-60,-12),Math.toRadians(180));


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(-37, -64.25), Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        armPIDController = new ArmPIDController(drive.armMotor);
        drive.tipCenter();
        drive.closeClaw();

        State currentState;

        Trajectory startToFirstDeposit = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-12, -62), Math.toRadians(90))
                .splineTo(new Vector2d(xCenterJunction, yCenterJunction), Math.toRadians(45))
                .build();

        Trajectory forward = drive.trajectoryBuilder(startToFirstDeposit.end())
                .forward(2.5)
                .build();

        Trajectory backUpFromJunction = drive.trajectoryBuilder(forward.end())
                .back(6)
                .build();

        Trajectory parkRed = drive.trajectoryBuilder(backUpFromJunction.end().minus(new Pose2d(0, 0, Math.toRadians(-45))))
                .lineToLinearHeading(new Pose2d(new Vector2d(-36, -36), Math.toRadians(90)))

                // .splineToLinearHeading(new Pose2d(new Vector2d(11.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
                .build();

        Trajectory parkGray = drive.trajectoryBuilder(backUpFromJunction.end().minus(new Pose2d(0, 0, Math.toRadians(-45))))
                .lineToLinearHeading(new Pose2d(new Vector2d(-62, -36), Math.toRadians(90)))
                // .strafeTo(new Vector2d(62,-37))
                .build();

        waitForStart();
        drive.closeClaw();
        drive.tipCenter();


        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }
        CV.stopSleeveCamera();

        currentState = State.SCORE_1;
        drive.followTrajectoryAsync(startToFirstDeposit);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
                case SCORE_1:
                    if (!drive.isBusy()) {
                       currentState= State.FORWARD;
                       drive.followTrajectoryAsync(forward);
                    } else {
                        armTarget = ARM_MID_TOP;
                        if (drive.armMotor.getCurrentPosition() > 100) {
                            liftTarget = SLIDE_HIGH;
                            drive.tipFront();
                            drive.closeClaw();
                        }
                    }
                    break;
                case FORWARD:
                    if (!drive.isBusy()){
                        sleep(300);
                        drive.openClaw();
                        sleep(300);
                        drive.closeClaw();
                        currentState = State.BACK_UP_FROM_JUNCTION;
                        drive.followTrajectoryAsync(backUpFromJunction);
                    }
                    break;
                case BACK_UP_FROM_JUNCTION:
                    if (!drive.isBusy()) {

                        currentState = State.TURN;
                        drive.turnAsync(Math.toRadians(47));
                    }
                    break;
                case TURN:
                    if (!drive.isBusy()) {
                        liftTarget = 0;
                        // drive.closeClaw();
                        // currentState = State.DONE;
                        if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.RED) {
                            drive.followTrajectoryAsync(parkRed);
                            currentState = State.PARK_RED;
                        } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {

                            currentState = State.PARK_GREEN;
                        } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GRAY) {
                            drive.followTrajectoryAsync(parkGray);
                            currentState = State.PARK_GRAY;
                        }
//                        currentState= State.PICKUP;
//                        drive.followTrajectoryAsync(firstDepositToConeStack1);
                    }
                    break;
                case NEW_CONE_FROM_SCORE_1:
                    telemetry.addData("Set up to grab new cone", "");
                    telemetry.update();
                    if (!drive.isBusy()) {


                        currentState = State.CONE_STACK1;
//                        drive.closeClaw();
//
//                        currentState = State.SCORE_CONE;
                        //     drive.followTrajectoryAsync(toConeStack2);
                    } else {
                        //liftTarget = 75;

                        armTarget = 110;
                        drive.openClaw();
                        drive.tipCenter();

                    }
                    break;
                case CONE_STACK1:
                    if (!drive.isBusy()) {
                        drive.closeClaw();
                        sleep(300);
                        currentState = State.PICKUP;
                        liftTarget = 300;
                    }
                    break;
                case PICKUP:
                    drive.closeClaw();
                    //liftTarget = 200;
                    if (drive.linearSlide.getCurrentPosition() > 250) {
                        currentState = State.SCORE_CONE;
                        armTarget = ARM_BACK;
                        liftTarget = SLIDE_HIGH;
                        // drive.followTrajectoryAsync(scoreNewCone);
                    }
                    break;
//                case BACK_UP:
//                    if (!drive.isBusy()){
//                        armTarget = ARM_BACK;
//                        drive.followTrajectory(scoreNewCone);
//                        currentState= State.SCORE_CONE;
//                    }
//                    break;


                case SCORE_CONE:

                    if (!drive.isBusy()) {
                        drive.openClaw();
                        sleep(500);
                        currentState = State.PARK_GREEN;
                    }
//                        time = new Date().getTime() - startTime;
//                        if (time< 25_000){
//                            //go get other cone
//                            currentState = State.NEW_CONE_FROM_SCORE_1;
//                            drive.followTrajectoryAsync(fromScoreNewConeToConeStack);
//
//                        } else {
//                            //go park
//                            if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GRAY) {
//                                drive.followTrajectoryAsync(parkGray);
//                                currentState = State.PARK_GRAY;
//                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {
//                                drive.followTrajectoryAsync(parkRed);
//                                currentState = State.PARK_GREEN;
//                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.RED) {
//                                currentState = State.PARK_RED;
//                            }
//                        }
//                    } else {
//                        if (drive.armMotor.getCurrentPosition()>100){
//                            liftTarget = SLIDE_HIGH;
//                        }
//                    }
                    break;
//                case NEW_CONE:
//                    if (!drive.isBusy()) {
//                        currentState = State.SCORE_CONE;
//                        drive.followTrajectoryAsync(scoreNewCone);
//                    } else if(getRuntime() == 900000000) {
//
//                    }
//                    break;
                case PARK_GRAY:
                case PARK_RED:
                case PARK_GREEN:

                    liftTarget = 0;
                    if (drive.linearSlide.getCurrentPosition() < 100) {
                        armTarget = 0;
                    }
                    if (!drive.isBusy()) {
                        if (drive.armMotor.getCurrentPosition() < 50) {
                            drive.openClaw();
                            drive.tipCenter();
                        }
                    }
                    break;
//                case PARK_RED:
//
//                    liftTarget= 0;
//                    if (drive.linearSlide.getCurrentPosition()<100){
//                        armTarget = 0;
//                    }
//                    if (!drive.isBusy()) {
//                        if (drive.armMotor.getCurrentPosition()<50){
//                            drive.openClaw();
//                            drive.tipCenter();
//                        }
//                    }
//                    break;
//                case PARK_GREEN:
//                    liftTarget= 0;
//                    if (drive.linearSlide.getCurrentPosition()<100){
//                        armTarget = 0;
//                    }
//                    if (drive.armMotor.getCurrentPosition()<50){
//                        drive.openClaw();
//                        drive.tipCenter();
//                    }
//                    break;
                case DONE:
                    break;
            }

            armPIDController.setTarget(armTarget);
            drive.armMotor.setPower(armPIDController.calculateVelocity());

            liftPIDController.setTarget(liftTarget);

            double power = liftPIDController.calculatePower();

            drive.linearSlide.setPower(power);
            drive.frontSlide.setPower(power);
            drive.slideOtherer.setPower(power);

            //  telemetry.addData("power ", power);
            telemetry.addData("arm target", armTarget);
            telemetry.addData("arm position", drive.armMotor.getCurrentPosition());
            telemetry.addData("lift target", liftTarget);
            telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());

            telemetry.update();

//        drive.closeClaw();
//
//        drive.setArmServoMiddle();
//
//        drive.liftTop();
//
//        TrajectorySequence startTo270Pole = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d( new Vector2d(- 11,-54), Math.toRadians(90)), Math.toRadians(90))
//                .lineToLinearHeading(new Pose2d(new Vector2d(-11,-30.5),Math.toRadians(45)))
//                .build();
//        drive.followTrajectorySequence(startTo270Pole);
//
//
//        //use vision to align
//
//        //drop cone
//        drive.liftMiddle();
//        sleep(1000);
//        drive.openClaw();
//        sleep(300);
//        drive.liftTop();
//
//        TrajectorySequence back = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .back(2)
//                .build();
//        drive.followTrajectorySequence(back);
//        drive.turn(Math.toRadians(45));
//
//        drive.setArmServoTop();
//        drive.liftDown();
//        while (this.opModeIsActive() && (drive.linearSlide.getCurrentPosition()>200|| drive.frontSlide.getCurrentPosition()>200)){
//
//        }
//        drive.setArmServoBottom();
//
//        TrajectorySequence secondCone = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .splineToLinearHeading(new Pose2d(new Vector2d(-11,-12),Math.toRadians(150)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(new Vector2d(-50,-12),Math.toRadians(180)), Math.toRadians(180))
//                .build();
//        drive.followTrajectorySequence(secondCone);
//
//        drive.openClaw();
//
//        TrajectorySequence score = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .splineToLinearHeading(new Pose2d(new Vector2d(-11,-12),Math.toRadians(180)), Math.toRadians(180))
//                .build();
//        drive.followTrajectorySequence(score);
//
//
//        TrajectorySequence newCone = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .splineToLinearHeading(new Pose2d(new Vector2d(-50,-12),Math.toRadians(180)), Math.toRadians(180))
//                .build();
//        drive.followTrajectorySequence(newCone);
//
//        drive.followTrajectorySequence(score);
//
//        drive.followTrajectorySequence(newCone);
//
//        drive.followTrajectorySequence(score);
//
//        drive.followTrajectorySequence(newCone);
//
//        drive.followTrajectorySequence(score);
//
//        drive.turn(90);
//
//
//        switch (sleeveColor) {
//            case GRAY:
//                //Parking 1
//                TrajectorySequence park1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .strafeTo(new Vector2d(-64, -12))
//                        .build();
//                drive.followTrajectorySequence(park1);
//                break;
//            case RED:
//                //Parking 2
//                TrajectorySequence park2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .strafeTo(new Vector2d(-38, -12))
//                        .build();
//                drive.followTrajectorySequence(park2);
//                break;
//            case GREEN:
//                //Parking 3
//                break;
//            case INDETERMINATE:
//                break;
//
//        }
//
//
//        // sleep(1000);
//
//        sleep (200);


//        TrajectorySequence toConeStack = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
//                .lineToLinearHeading(coneStack)
//                .build();
//        drive.followTrajectorySequence(toConeStack);
//
//
//        TrajectorySequence toWestPole = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
//                .lineToLinearHeading(westPoleDeposit)
//                .build();
//        drive.followTrajectorySequence(toWestPole);
//

            //park in the correct spot
//        drive.followTrajectorySequence(toConeStack);

            //put lift down

        }

    }
}
