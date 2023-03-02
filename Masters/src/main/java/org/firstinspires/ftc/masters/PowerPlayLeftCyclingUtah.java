package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_CONE_STACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH_AUTO;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_MIDDLE;
import static org.firstinspires.ftc.masters.BadgerConstants.STACK_OFFSET;

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
@Autonomous(name = "Power Play Left Cycling Utah", group = "competition")
public class PowerPlayLeftCyclingUtah extends LinearOpMode {

    Trajectory cyclePickupCone2;

    enum State {
        FIRST_DEPOSIT_PATH_1,
        FIRST_DEPOSIT_SCORE_CONE,

        BACK_UP_FROM_JUNCTION,

        CYCLE_PICKUP_TURN,
        CYCLE_PICKUP_PATH1,
        CYCLE_PICKUP_PATH2,
        CYCLE_PICKUP_END,

        CYCLE_SCORE_PATH1,
        CYCLE_SCORE_TURN,
        CYCLE_SCORE_ALIGN,
        CYCLE_SCORE_CONE,
        CYCLE_BACK_UP,

        PICKUP,
        BACK_UP,
        ALIGN,

        PARK_GRAY,
        PARK_RED,
        PARK_GREEN,
        DONE,
        LIFT,
        FINISH

    }

    LiftPIDController liftPIDController;
    ArmPIDControllerMotionProfile armPIDController;

    int armTarget = 0, liftTarget = 0;

    public static double xStack = -63;
    public static double yStack = -13;

    public static int turnJunction = 45;
    boolean retractArm = false;

    int coneStack = ARM_CONE_STACK;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(-37, -64.25), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        // liftPIDController.setP(0.015);
        armPIDController = new ArmPIDControllerMotionProfile(drive.armMotor);
        drive.tipCenter();
        drive.closeClaw();

        State currentState;

        // Trajectory from start to nearest tall pole
        Trajectory firstDepositPath1 = drive.trajectoryBuilder(startPose)

                .lineTo(new Vector2d(-37, -35))
                .splineToSplineHeading(new Pose2d(-36, -14, Math.toRadians(45)), Math.toRadians(45))

                .build();

        Trajectory firstDepositScoreCone;

        Trajectory backUpFromJunction = drive.trajectoryBuilder(firstDepositPath1.end())
                .back(3)
                .build();


        Trajectory cyclePickupPath1 = drive.trajectoryBuilder(backUpFromJunction.end().plus(new Pose2d(0, 0, Math.toRadians(turnJunction + 90))))
                .splineToLinearHeading(new Pose2d(new Vector2d(xStack, yStack), Math.toRadians(180)), Math.toRadians(180))
                .build();


        Trajectory cycleDepositScoreCone = drive.trajectoryBuilder(cyclePickupPath1.end())
                .lineTo(new Vector2d(-45, -12))
                .splineToSplineHeading(new Pose2d(-38, -13, Math.toRadians(45+180)), Math.toRadians(45))

                .build();

        Trajectory cycleBackUpFromJunction = drive.trajectoryBuilder(cycleDepositScoreCone.end())
                .back(6)
                .build();


        Trajectory cyclePickupCone = drive.trajectoryBuilder(cyclePickupPath1.end())
                .splineToLinearHeading(new Pose2d(new Vector2d(-45, -12),Math.toRadians(180)),Math.toRadians(180))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(cyclePickupCone2))
                .build();

        cyclePickupCone2 = drive.trajectoryBuilder(cyclePickupCone.end())
                .lineTo(new Vector2d(xStack, yStack))
                        .build();

        Trajectory parkGray1, parkGray2, parkRed, parkGreen1, parkGreen2;



        waitForStart();

        drive.closeClaw();
        drive.tipCenter();

        long startTime = new Date().getTime();
        long time = 0;

        long alignTime = new Date().getTime();

        while (time < 100 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }
        CV.setPipeDetectionFront();

        currentState = State.FIRST_DEPOSIT_PATH_1;
        drive.followTrajectoryAsync(firstDepositPath1);
        armTarget = ARM_MID_TOP;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
                case FIRST_DEPOSIT_PATH_1:
                    if (!drive.isBusy()) {
                        currentState = State.ALIGN;
                        alignTime = new Date().getTime();
                    } else {
                        if (drive.armMotor.getCurrentPosition() > 100) {
                            liftTarget = SLIDE_HIGH_AUTO;
                            drive.tipFront();
                        }
                    }
                    break;

                case ALIGN:
                    if (drive.alignPole(CV.sleevePipeline.position) || new Date().getTime() - alignTime > 1000) {
                        telemetry.addData("done aligning", "score cone");
                        currentState = State.FIRST_DEPOSIT_SCORE_CONE;
                        firstDepositScoreCone = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(6)
                                .build();
                        drive.followTrajectoryAsync(firstDepositScoreCone);
                    }
                    break;
                case FIRST_DEPOSIT_SCORE_CONE:
                    if (!drive.isBusy()) {
                        drive.openClaw();
                        sleep(350);
                        drive.closeClaw();
                        drive.followTrajectoryAsync(backUpFromJunction);
                        currentState = State.BACK_UP_FROM_JUNCTION;
                        CV.sleeveWebcam.stopStreaming();
                    }
                    break;
                case BACK_UP_FROM_JUNCTION:
                    if (!drive.isBusy()) {
                        currentState = State.CYCLE_PICKUP_TURN;
                        drive.turnAsync(Math.toRadians(turnJunction + 90));

                    } else {
                        liftTarget = 0;
                        armTarget = coneStack;
                        if (drive.linearSlide.getCurrentPosition() < 100) {
                            drive.openClaw();
                            drive.tipCenter();
                        }

                    }
                    break;
                case CYCLE_PICKUP_TURN:
                    if (!drive.isBusy()){
                        currentState = State.CYCLE_PICKUP_PATH1;
                        drive.followTrajectory(cyclePickupPath1);
                    } else{
                        liftTarget = 0;
                        armTarget = coneStack;
                        if (drive.linearSlide.getCurrentPosition() < 100) {
                            drive.openClaw();
                            drive.tipCenter();
                        }
                    }
                    break;

                case CYCLE_PICKUP_PATH1:
                    if (!drive.isBusy()) {
                        drive.closeClaw();
                        sleep(300);
                        liftTarget = SLIDE_MIDDLE;
                        currentState = State.LIFT;
                    }
                    break;

                case LIFT:
                    if (drive.linearSlide.getCurrentPosition() > SLIDE_MIDDLE - 30) {
                        drive.followTrajectoryAsync(cycleDepositScoreCone);
                        currentState = State.CYCLE_SCORE_PATH1;
                        coneStack = coneStack - STACK_OFFSET;

                    }
                    break;
                case CYCLE_SCORE_PATH1:
                    if (!drive.isBusy()) {
                        currentState = State.CYCLE_SCORE_ALIGN;
                        alignTime = new Date().getTime();
                    } else {
                        liftTarget = SLIDE_HIGH_AUTO;
                        armTarget = ARM_BACK_TOP;

                        if (drive.armMotor.getCurrentPosition() > 1000) {
                            drive.tipBack();
                        }
                    }
                    break;


                case CYCLE_SCORE_ALIGN:
                    if (drive.alignPole(CV.pipeDetectionPipeline.position) || new Date().getTime() - alignTime > 1000) {
                        currentState = State.CYCLE_SCORE_CONE;
                        cycleDepositScoreCone = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(1)
                                .build();
                        drive.followTrajectoryAsync(cycleDepositScoreCone);
                    }
                    break;
                case CYCLE_SCORE_CONE:
                    if (!drive.isBusy()) {
                        drive.openClaw();
                        cycleBackUpFromJunction = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(4)
                                .build();
                        sleep(300);
                        drive.followTrajectoryAsync(cycleBackUpFromJunction);
                        currentState = State.CYCLE_BACK_UP;

                    }
                    break;
                case CYCLE_BACK_UP:
                    if (!drive.isBusy()) {

                        time = new Date().getTime() - startTime;
                        if (time >20_000){
                            liftTarget = 0;
                            armTarget = coneStack;
                            if (drive.linearSlide.getCurrentPosition() < 100) {
                                drive.openClaw();
                                drive.tipCenter();
                            }
                            drive.turnAsync(Math.toRadians(45));
                            liftTarget = 0;
                            switch (sleeveColor){
                                case GRAY:
                                    parkGray1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(new Vector2d(-60, -12), Math.toRadians(270)))
                                            .build();
                                    drive.followTrajectoryAsync(parkGray1);
                                    currentState = State.PARK_GRAY;
                                    break;
                                case RED:
                                    parkRed = drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(new Vector2d(-34, -24),Math.toRadians(270)))
                                            .build();
                                    drive.followTrajectoryAsync(parkRed);
                                    currentState = State.PARK_RED;
                                    break;
                                case GREEN:
                                    parkGreen1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d( new Vector2d(-12, -12), Math.toRadians(270)))
                                            .build();
                                    drive.followTrajectoryAsync(parkGreen1);
                                    currentState = State.PARK_GREEN;
                                    break;
                            }
                            //go park
                        } else {
                            if (drive.linearSlide.getCurrentPosition() < 100) {
                                drive.openClaw();
                                drive.tipCenter();
                            }
                            drive.followTrajectory(cyclePickupCone);
                            currentState= State.CYCLE_PICKUP_PATH1;
                        }


                    } else {
                        drive.closeClaw();
                        liftTarget = 0;
                        armTarget = coneStack;
                        if (drive.linearSlide.getCurrentPosition() < 100) {
                            drive.openClaw();
                            drive.tipCenter();
                        }
                    }
                    break;

                case PARK_GRAY:

                    liftTarget = 0;
                    if (drive.linearSlide.getCurrentPosition() < 100) {
                        armTarget = 0;
                    }
                    if (!drive.isBusy()) {
//                        parkGray2 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(new Vector2d(-58, -24), Math.toRadians(270)))
//                                .build();
//                        drive.followTrajectoryAsync(parkGray2);
                        currentState = State.DONE;

                    }

                case PARK_RED:
                    liftTarget = 0;
                    if (drive.linearSlide.getCurrentPosition() < 100) {
                        armTarget = 0;
                    }
                    if (!drive.isBusy()) {
                        currentState = State.DONE;

                    }
                    break;

                case PARK_GREEN:
                    liftTarget = 0;
                    if (drive.linearSlide.getCurrentPosition() < 100) {
                        armTarget = 0;
                    }
                    if (!drive.isBusy()) {
                        parkGreen2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d( new Vector2d(-12, -24), Math.toRadians(270)))
                                .build();
                        drive.followTrajectoryAsync(parkGreen2);
                        currentState = State.DONE;

                    }


                    break;

                case DONE:
                    if (!drive.isBusy()) {
                        if (drive.armMotor.getCurrentPosition() < 50) {
                            drive.openClaw();
                            drive.tipCenter();
                        }

                    }

            }


            armPIDController.setTarget(armTarget);
            drive.armMotor.setVelocity(armPIDController.calculateVelocity());

            liftPIDController.setTarget(liftTarget);
            double power = liftPIDController.calculatePower(drive.linearSlide);
            drive.linearSlide.setPower(power);
            drive.slideOtherer.setPower(liftPIDController.calculatePower(drive.slideOtherer));
            drive.frontSlide.setPower(power);

//            liftPIDController.setTarget(liftTarget);
//
//            double power = liftPIDController.calculatePower();
//            double powerLeft= liftPIDController.calculatePower(drive.slideOtherer);
//
//            drive.linearSlide.setPower(power);
//            drive.frontSlide.setPower(power);
//            drive.slideOtherer.setPower(powerLeft);

            //  telemetry.addData("power ", power);
            telemetry.addData("arm target", armTarget);
//            telemetry.addData("arm position", drive.armMotor.getCurrentPosition());
            telemetry.addData("lift target", liftTarget);
            // telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());

            //  telemetry.update();

        }


    }

}
