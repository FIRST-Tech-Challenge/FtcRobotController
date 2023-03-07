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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.drive.SampleMecanumDriveCancelable;

import java.util.Date;

@Config
//@Autonomous(name = "Power Play Right with pause", group = "competition")
public class PowerPlayRightWithPause extends LinearOpMode {

    enum State {
        SCORE_1,
        FORWARD,
        ALIGN,
        BACK_UP_FROM_JUNCTION,
        TURN,
        PICKUP,
        BACK_UP,
        PARK_GRAY,
        PARK_RED,
        PARK_GREEN,
        DONE

    }

    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    int armTarget=0, liftTarget =0;

    public static  double xCenterJunction = 8;
    public static double yCenterJunction =-33;

    public static int turnJunction = 45;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;


        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(new Vector2d(36, -64.25), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        armPIDController = new ArmPIDController(drive.armMotor);
        drive.tipCenter();
        drive.closeClaw();

        State currentState;

        // Trajectory from start to nearest tall pole
        Trajectory startToFirstDeposit = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(12,-62),Math.toRadians(90))
                .splineTo(new Vector2d(xCenterJunction, yCenterJunction),Math.toRadians(90+turnJunction))
                .build();

        Trajectory forward;

        Trajectory backUpFromJunction;
        Trajectory parkRed = null;
        Trajectory parkGreen = null;

        CV.setSleevePipeline();

        waitForStart();

        drive.closeClaw();
        drive.tipCenter();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 6200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }
        CV.setPipeDetectionFront();

        currentState = State.SCORE_1;
        drive.followTrajectoryAsync(startToFirstDeposit);

        while(opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
                case SCORE_1:
                    if (!drive.isBusy()) {
                        currentState= State.ALIGN;
                        drive.alignPole(CV.sleevePipeline.position);
                    } else {
                        armTarget = ARM_MID_TOP;
                        if (drive.armMotor.getCurrentPosition()>100){
                            liftTarget= SLIDE_HIGH_AUTO;
                            drive.tipFront();
                            drive.closeClaw();
                        }
                    }
                    break;
                case ALIGN:
                    if (drive.alignPole(CV.sleevePipeline.position)){
                        currentState = State.FORWARD;
                        forward = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(2.5)
                                .build();
                        drive.followTrajectoryAsync(forward);
                    }

                case FORWARD:
                    if (!drive.isBusy()){
                        backUpFromJunction = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(6)
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
                    if (!drive.isBusy()){

                        currentState= State.TURN;
                        drive.turnAsync(Math.toRadians(-turnJunction));
                    }
                    break;
                case TURN:
                    if (!drive.isBusy()){
                        liftTarget = 0;

                        if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.RED) {
                            parkRed = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(new Vector2d(36,-38),Math.toRadians(90)))
                                    .build();
                            drive.followTrajectoryAsync(parkRed);
                            currentState = State.PARK_RED   ;
                        } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {
                            parkGreen = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(new Vector2d(62,-38 ),Math.toRadians(90)))
                                    .build();
                            drive.followTrajectoryAsync(parkGreen);
                            currentState = State.PARK_GREEN;
                        } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GRAY) {
                            currentState = State.PARK_GRAY;
                        }
                    }
                    break;

                case PARK_GRAY:
                case PARK_RED:
                case PARK_GREEN:

                    liftTarget= 0;
                    if (drive.linearSlide.getCurrentPosition()<100){
                        armTarget = 0;
                    }
                    if (!drive.isBusy()) {
                        if (drive.armMotor.getCurrentPosition()<50){
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
            telemetry.addData("arm position", drive.armMotor.getCurrentPosition() );
            telemetry.addData("lift target", liftTarget);
            telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());

            telemetry.update();

            PositionStorage.armPosition = drive.armMotor.getCurrentPosition();
            PositionStorage.liftPosition = drive.linearSlide.getCurrentPosition();
            PositionStorage.currentPose = drive.getPoseEstimate();

        }



    }

}
