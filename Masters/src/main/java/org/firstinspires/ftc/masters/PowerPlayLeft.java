package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH_AUTO;
import static org.firstinspires.ftc.masters.PowerPlayLeft.State.ALIGN;
import static org.firstinspires.ftc.masters.PowerPlayLeft.State.FORWARD;

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
@Autonomous(name = "LEFT preload only WORLD", group="competition")
public class PowerPlayLeft extends LinearOpMode {

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

        Trajectory start = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-10, -62), Math.toRadians(90))

                .build();

        Trajectory straight = drive.trajectoryBuilder(start.end())
                .lineTo(new Vector2d(xCenterJunction, yCenterJunction))
                .build();

        Trajectory forward = drive.trajectoryBuilder(start.end())
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
        drive.followTrajectoryAsync(start);

        int slidePosition = drive.linearSlide.getCurrentPosition();
        int armPosition = drive.armMotor.getCurrentPosition();
        long alignTime = new Date().getTime();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
                case SCORE_1:
                if (!drive.isBusy()) {
                    drive.followTrajectoryAsync(straight);
                    currentState = State.SCORE_2;
                } else {
                    armTarget = ARM_MID_TOP;
                    if (drive.armMotor.getCurrentPosition() > 100) {
                        liftTarget = SLIDE_HIGH_AUTO;
                        drive.tipFront();
                        drive.closeClaw();
                    }
                }

                break;
                case SCORE_2:
                    if (!drive.isBusy()){
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
                    if (drive.alignPole(CV.sleevePipeline.position)){
                        currentState = FORWARD;
                        forward = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(8)
                                .build();
                        drive.followTrajectoryAsync(forward);
                    }
                    break;
                case FORWARD:
                    if (!drive.isBusy()){
                        backUpFromJunction = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(10)
                                .build();
                      //  sleep(100);
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
            //  telemetry.addData("power ", power);
//            telemetry.addData("arm target", armTarget);
//            telemetry.addData("arm position", drive.armMotor.getCurrentPosition());
//            telemetry.addData("lift target", liftTarget);
//            telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());
//
//            PositionStorage.armPosition = drive.armMotor.getCurrentPosition();
//            PositionStorage.liftPosition = drive.linearSlide.getCurrentPosition();
//            PositionStorage.currentPose = drive.getPoseEstimate();
//
//
//            telemetry.update();



        }

    }
}
