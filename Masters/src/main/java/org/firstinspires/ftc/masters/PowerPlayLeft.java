package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH_AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;
import java.util.List;

@Config
@Autonomous(name = "Power Play Left", group="competition")
public class PowerPlayLeft extends LinearOpMode {

    enum State {
        SCORE_1,
        FORWARD,
        ALIGN,
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

    public static int turnJunction = 45;


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        CV.setSleevePipeline();
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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
                        currentState= State.ALIGN;
                        drive.alignPole(CV.sleevePipeline.position);
                    } else {
                        armTarget = ARM_MID_TOP;
                        if (drive.armMotor.getCurrentPosition() > 100) {
                            liftTarget = SLIDE_HIGH_AUTO;
                            drive.tipFront();
                            drive.closeClaw();
                        }
                    }
                    break;
                case ALIGN:
                    if (drive.alignPole(CV.sleevePipeline.position)){
                        currentState = State.FORWARD;
                        forward = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(8)
                                .build();
                        drive.followTrajectoryAsync(forward);
                    }
                case FORWARD:
                    if (!drive.isBusy()){
                        backUpFromJunction = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(10)
                                .build();
                        sleep(100);
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


                case SCORE_CONE:

                    if (!drive.isBusy()) {
                        drive.openClaw();
                        sleep(500);
                        currentState = State.PARK_GREEN;
                    }

                    break;

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

                case DONE:
                    break;
            }

            armPIDController.setTarget(armTarget);
            drive.armMotor.setPower(armPIDController.calculateVelocity());

//            liftPIDController.setTarget(liftTarget);
//
//            double power = liftPIDController.calculatePower();
//            double powerLeft= liftPIDController.calculatePower(drive.slideOtherer);
//
//            drive.linearSlide.setPower(power);
//            drive.frontSlide.setPower(power);
//            drive.slideOtherer.setPower(powerLeft);
            drive.linearSlide.setTargetPosition(liftTarget);
            drive.frontSlide.setTargetPosition(liftTarget);
            drive.slideOtherer.setTargetPosition(liftTarget);
            drive.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.linearSlide.setPower(1);
            drive.frontSlide.setPower(1);
            drive.slideOtherer.setPower(1);

            //  telemetry.addData("power ", power);
            telemetry.addData("arm target", armTarget);
            telemetry.addData("arm position", drive.armMotor.getCurrentPosition());
            telemetry.addData("lift target", liftTarget);
            telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());

            PositionStorage.armPosition = drive.armMotor.getCurrentPosition();
            PositionStorage.liftPosition = drive.linearSlide.getCurrentPosition();
            PositionStorage.currentPose = drive.getPoseEstimate();


            telemetry.update();



        }

    }
}
