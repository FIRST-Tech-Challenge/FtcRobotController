package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;

@Autonomous(name = "Power Play Right")
public class PowerPlayRight extends LinearOpMode {

    enum State {
        SCORE_1,
        SCORE_2,
        PICKUP,
        BACK_UP,
        NEW_CONE_FROM_SCORE_1,
        SCORE_CONE,
        NEW_CONE,
        PARK_GRAY,
        PARK_RED,
        PARK_GREEN

    }

    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    int armTarget=0, liftTarget =0;

    int numberOfConesPickedUp =0;

    @Override
    public void runOpMode() {

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(37.5, -64.25), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        armPIDController = new ArmPIDController(drive.armMotor);
        drive.tipCenter();
        drive.closeClaw();

        State currentState;

        // Trajectory from start to nearest tall pole
        Trajectory startToFirstDeposit = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(14,-60),Math.toRadians(90))
                .splineTo(new Vector2d(10.75,-35.5),Math.toRadians(135))
                .build();

        Trajectory firstDepositToConeStack = drive.trajectoryBuilder(startToFirstDeposit.end())
                .splineTo(new Vector2d(20,-14), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(59, -14))
                .build();

        Trajectory conePickup = drive.trajectoryBuilder(firstDepositToConeStack.end())
                .lineTo(new Vector2d(49, -14))
                .build();

        Trajectory scoreNewCone = drive.trajectoryBuilder(conePickup.end())
                .splineTo(new Vector2d(32.5,-11.5),Math.toRadians(315))
                .build();

        Trajectory fromScoreNewConeToConeStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(59,-14),Math.toRadians(0)))
                .build();

        Trajectory parkGray = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(new Vector2d(11.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
                .build();

        Trajectory parkRed = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(new Vector2d(23.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
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

        currentState = State.SCORE_1;
        drive.followTrajectoryAsync(startToFirstDeposit);

        while(opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
                case SCORE_1:
                    if (!drive.isBusy()) {
                        drive.openClaw();
                        sleep(500);
                        drive.closeClaw();
                        currentState = State.NEW_CONE_FROM_SCORE_1;
                        drive.followTrajectoryAsync(firstDepositToConeStack);
                    } else {
                        armTarget = ARM_MID_TOP;
                        if (drive.armMotor.getCurrentPosition()>100){
                            liftTarget= SLIDE_HIGH;
                            drive.tipFront();
                        }
                    }
                    break;
                case NEW_CONE_FROM_SCORE_1:
                    telemetry.addData("Set up to grab new cone","");
                    telemetry.update();
                    if (!drive.isBusy()) {

                        currentState = State.PICKUP;
//                        drive.closeClaw();
//
//                        currentState = State.SCORE_CONE;
//                        drive.followTrajectoryAsync(scoreNewCone);
                    } else {
                        liftTarget = 75;
                        if (drive.linearSlide.getCurrentPosition()<100){
                            armTarget = 0;
                            drive.openClaw();
                            drive.tipCenter();
                        }
                    }
                    break;
                case PICKUP:
                    drive.closeClaw();
                    liftTarget = 150;
                    if (drive.linearSlide.getCurrentPosition()>140){
                        currentState = State.BACK_UP;
                        drive.followTrajectoryAsync(conePickup);
                    }
                    break;
                case BACK_UP:
                    if (!drive.isBusy()){
                        armTarget = ARM_BACK;
                        drive.followTrajectory(scoreNewCone);
                        currentState= State.SCORE_CONE;
                    }
                    break;


                case SCORE_CONE:

                    if (!drive.isBusy()){
                        drive.openClaw();
                        sleep(500);
                        time = new Date().getTime() - startTime;
                        if (time< 25_000){
                            //go get other cone
                            currentState = State.NEW_CONE_FROM_SCORE_1;
                            drive.followTrajectoryAsync(fromScoreNewConeToConeStack);

                        } else {
                            //go park
                            if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GRAY) {
                                drive.followTrajectoryAsync(parkGray);
                                currentState = State.PARK_GRAY;
                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {
                                drive.followTrajectoryAsync(parkRed);
                                currentState = State.PARK_GREEN;
                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.RED) {
                                currentState = State.PARK_RED;
                            }
                        }
                    } else {
                        if (drive.armMotor.getCurrentPosition()>100){
                            liftTarget = SLIDE_HIGH;
                        }
                    }
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
                case PARK_RED:

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
                case PARK_GREEN:
                    liftTarget= 0;
                    if (drive.linearSlide.getCurrentPosition()<100){
                        armTarget = 0;
                    }
                    if (drive.armMotor.getCurrentPosition()<50){
                        drive.openClaw();
                        drive.tipCenter();
                    }
                    break;
            }
        }

        armPIDController.setTarget(armTarget);
        drive.armMotor.setVelocity(armPIDController.calculateVelocity());

        liftPIDController.setTarget(liftTarget);

        drive.linearSlide.setPower(liftPIDController.calculatePower());


    }

}
