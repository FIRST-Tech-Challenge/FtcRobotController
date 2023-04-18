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
import org.firstinspires.ftc.masters.drive.SampleMecanumDriveCancelable;

import java.util.Date;
import java.util.List;

@Config
@Autonomous(name = "RIGHT preload only world", group = "competition")
public class PowerPlayRight extends LinearOpMode {

    enum State {
        SCORE_1,
        SCORE_2,
        TURN_1,
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

    public static  double xCenterJunction = 7;
    public static double yCenterJunction =-34;

    public static int turnJunction = 45;

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
        Trajectory start = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(10,-62),Math.toRadians(90))
                //.splineTo(new Vector2d(xCenterJunction, yCenterJunction),Math.toRadians(90+turnJunction))
                .build();

        Trajectory straight = drive.trajectoryBuilder(start.end())
                .lineTo(new Vector2d(xCenterJunction, yCenterJunction))
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

        int slidePosition = drive.linearSlide.getCurrentPosition();
        int armPosition = drive.armMotor.getCurrentPosition();

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }
        CV.setPipeDetectionFront();

        currentState = State.SCORE_1;
        drive.followTrajectoryAsync(start);
        long alignTime = new Date().getTime();

        while(opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
                case SCORE_1:
                    if (!drive.isBusy()) {
                       drive.followTrajectoryAsync(straight);
                       currentState= State.SCORE_2;
                    } else {
                        armTarget = ARM_MID_TOP;
                        if (drive.armMotor.getCurrentPosition()>100){
                            liftTarget= SLIDE_HIGH_AUTO;
                            drive.tipFront();
                            drive.closeClaw();
                        }
                    }
                    break;
                case SCORE_2:if (!drive.isBusy()){
                    drive.turnAsync(Math.toRadians(turnJunction));
                    currentState= State.TURN_1;
                } else {
                    armTarget = ARM_MID_TOP;
                    if (drive.armMotor.getCurrentPosition()>100){
                        liftTarget= SLIDE_HIGH_AUTO;
                        drive.tipFront();
                        drive.closeClaw();
                    }
                }
                break;
                case TURN_1:if (!drive.isBusy()) {
                    currentState= State.ALIGN;
                    alignTime = new Date().getTime();
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
                    if (drive.alignPole(CV.sleevePipeline.position) || new Date().getTime()- alignTime >500){
                        currentState = State.FORWARD;
                        Trajectory trajForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(6)
                                .build();
                        drive.followTrajectoryAsync(trajForward);
                    }
                    break;

                case FORWARD:
                    if (!drive.isBusy()){
                        backUpFromJunction = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(6)
                                .build();
                        //sleep(100);
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
                                    .lineToLinearHeading(new Pose2d(new Vector2d(39,-37),Math.toRadians(90)))
                                    .build();
                                drive.followTrajectoryAsync(parkRed);
                                currentState = State.PARK_RED   ;
                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {
                                parkGreen = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(new Vector2d(65,-37 ),Math.toRadians(90)))
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



            armPosition = drive.armMotor.getCurrentPosition();

            armPIDController.setTarget(armTarget);
            drive.armMotor.setPower(armPIDController.calculateVelocity(armPosition));

            liftPIDController.setTarget(liftTarget);
            slidePosition = drive.linearSlide.getCurrentPosition();
            double power = liftPIDController.calculatePower(slidePosition);
            drive.linearSlide.setPower(power);
            drive.slideOtherer.setPower(liftPIDController.calculatePowerSingle(drive.slideOtherer.getCurrentPosition()));
            //drive.slideOtherer.setPower(power);
            drive.frontSlide.setPower(power);

//            liftPIDController.setTarget(liftTarget);
//            drive.linearSlide.setPower(liftPIDController.calculatePower(drive.linearSlide));
//            drive.slideOtherer.setPower(liftPIDController.calculatePower(drive.slideOtherer));
//            drive.frontSlide.setPower(liftPIDController.calculatePower(drive.frontSlide));

          //  telemetry.addData("power ", power);
//            telemetry.addData("arm target", armTarget);
//            telemetry.addData("arm position", drive.armMotor.getCurrentPosition() );
//            telemetry.addData("lift target", liftTarget);
//            telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());
//
//            telemetry.update();
//
//            PositionStorage.armPosition = drive.armMotor.getCurrentPosition();
//            PositionStorage.liftPosition = drive.linearSlide.getCurrentPosition();
//            PositionStorage.currentPose = drive.getPoseEstimate();

        }



    }

}
