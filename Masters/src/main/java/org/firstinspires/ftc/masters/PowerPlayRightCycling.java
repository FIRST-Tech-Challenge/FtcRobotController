package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_CONE_STACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_MIDDLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;

@Config
@Autonomous(name = "Power Play Right Cycling")
public class PowerPlayRightCycling extends LinearOpMode {

    enum State {
        SCORE_1,
        BACK_UP_FROM_JUNCTION,
        TURN,
        TURN_2_CONE,
        SCORE_2,
        CONE_STACK1,
        PICKUP,
        BACK_UP,
        NEW_CONE_FROM_SCORE_1,
        SCORE_CONE,
        SCORE_NEW_CONE,
        PARK_GRAY,
        PARK_RED,
        PARK_GREEN,
        DONE,
        LIFT

    }

    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    int armTarget = 0, liftTarget = 0;

    int numberOfConesPickedUp = 0;

    public static double xCenterJunction = 7;
    public static double yCenterJunction = -31;

    public static double xIntermediateStack = 20;
    public static double yIntermediateStack = -12;
    public static double angleIntermediateStack = 0;
    public static double xStack = 53;
    public static double yStack = -13;

    public static int turnJunction = 45;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(36, -64.25), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        armPIDController = new ArmPIDController(drive.armMotor);
        drive.tipCenter();
        drive.closeClaw();

        State currentState;

        // Trajectory from start to nearest tall pole
        Trajectory startToFirstDeposit = drive.trajectoryBuilder(startPose)

                .splineToConstantHeading(new Vector2d(36, -14), Math.toRadians(90))
                .build();


        Trajectory turnToJunction = drive.trajectoryBuilder(startToFirstDeposit.end().plus(new Pose2d(0, 0, Math.toRadians(turnJunction))))
                .lineTo(new Vector2d(31, -8))
                .build();

        Trajectory backUpFromJunction = drive.trajectoryBuilder(turnToJunction.end())
                .back(6)
                .build();

        Trajectory toConeStack1 = drive.trajectoryBuilder(backUpFromJunction.end().plus(new Pose2d(0,0,Math.toRadians(-turnJunction-90))))
                .lineTo(new Vector2d(xStack, yStack))
                .build();
        Trajectory toConeStackForward = drive.trajectoryBuilder(toConeStack1.end())
                .lineTo(new Vector2d(xStack+8.5, yStack))
                .build();

//        Trajectory conePickup = drive.trajectoryBuilder(toConeStack2.end())
//                .lineToConstantHeading(new Vector2d(49, -9))
//                .build();

        Trajectory scoreNewCone = drive.trajectoryBuilder(toConeStackForward.end())
                .back(25)
               // .lineTo(new Vector2d(36, -13))
                //.splineToLinearHeading(new Pose2d(new Vector2d(22, -9.5), 45), Math.toRadians(135))
                //  .splineTo(new Vector2d(30.5,-9.5),Math.toRadians(315))
                .build();

        Trajectory fromScoreNewConeToConeStack = drive.trajectoryBuilder(scoreNewCone.end())
                .lineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(0)))
                .build();

        Trajectory parkRed = drive.trajectoryBuilder(backUpFromJunction.end().minus(new Pose2d(0, 0, Math.toRadians(turnJunction))))
                .lineToLinearHeading(new Pose2d(new Vector2d(36, -38), Math.toRadians(90)))

                // .splineToLinearHeading(new Pose2d(new Vector2d(11.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
                .build();

        Trajectory parkGreen = drive.trajectoryBuilder(backUpFromJunction.end().minus(new Pose2d(0, 0, Math.toRadians(turnJunction))))
                .lineToLinearHeading(new Pose2d(new Vector2d(62, -38), Math.toRadians(90)))
                // .strafeTo(new Vector2d(62,-37))
                .build();

//        Trajectory parkRed = drive.trajectoryBuilder(scoreNewCone.end())
//                .splineToLinearHeading(new Pose2d(new Vector2d(23.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
//                .build();

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

                        currentState = State.TURN;
                        drive.turnAsync(Math.toRadians(turnJunction));
                    } else {
                        armTarget = ARM_MID_TOP;
                        if (drive.armMotor.getCurrentPosition() > 100) {
                            liftTarget = SLIDE_HIGH;
                            drive.tipFront();
                            drive.closeClaw();
                        }
                    }
                    break;
                case TURN:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(turnToJunction);
                        currentState = State.SCORE_CONE;
                    }
                    break;
                case SCORE_CONE:
                    if (!drive.isBusy()) {
                        sleep(300);
                        drive.openClaw();
                        sleep(300);
                        drive.closeClaw();
                        drive.followTrajectoryAsync(backUpFromJunction);
                        currentState = State.BACK_UP_FROM_JUNCTION;
                    }
                    break;
                case BACK_UP_FROM_JUNCTION:
                    if (!drive.isBusy()) {
                        currentState = State.TURN_2_CONE;
                        drive.turnAsync(Math.toRadians(-turnJunction-90));
                    }
                    break;
                case TURN_2_CONE:
                    if (!drive.isBusy()){
                        currentState = State.CONE_STACK1;
                        drive.followTrajectoryAsync(toConeStack1);
                    } else {
                        liftTarget = 0;
                        if (drive.linearSlide.getCurrentPosition() < 100) {
                            armTarget = 0;
                        }
                    }
                    break;
                case CONE_STACK1:
                    if (!drive.isBusy()){
                        armTarget = ARM_CONE_STACK;
                        drive.openClaw();
                        drive.tipCenter();
                        drive.followTrajectoryAsync(toConeStackForward);
                        currentState= State.PICKUP;
                    }
                    break;
                case PICKUP:
                    if (!drive.isBusy()){
                        drive.closeClaw();
                        sleep(300);
                        liftTarget= SLIDE_MIDDLE;
                        currentState = State.LIFT;
                    }
                    break;
                case LIFT:
                    if (drive.linearSlide.getCurrentPosition()>SLIDE_MIDDLE-30){
                        drive.followTrajectoryAsync(scoreNewCone);
                        currentState= State.SCORE_NEW_CONE;

                    }
                    break;
                case  SCORE_NEW_CONE:
                    if (drive.isBusy()){

                    } else {
                        liftTarget= SLIDE_HIGH;
                    }
                    break;
//                case TURN:
//                    if (!drive.isBusy()){
//                        liftTarget = 0;
//                       // drive.closeClaw();
//                       // currentState = State.DONE;
//                        if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.RED) {
//                                drive.followTrajectoryAsync(parkRed);
//                                currentState = State.PARK_RED   ;
//                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {
//                                drive.followTrajectoryAsync(parkGreen);
//                                currentState = State.PARK_GREEN;
//                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GRAY) {
//                                currentState = State.PARK_GRAY;
//                            }
////                        currentState= State.PICKUP;
////                        drive.followTrajectoryAsync(firstDepositToConeStack1);
//                    }
//                    break;
//                case NEW_CONE_FROM_SCORE_1:
//                    telemetry.addData("Set up to grab new cone","");
//                    telemetry.update();
//                    if (!drive.isBusy()) {
//
//
//                        currentState = State.CONE_STACK1;
////                        drive.closeClaw();
////
////                        currentState = State.SCORE_CONE;
//                       drive.followTrajectoryAsync(toConeStack2);
//                    } else {
//                        //liftTarget = 75;
//
//                            armTarget = 110;
//                            drive.openClaw();
//                            drive.tipCenter();
//
//                    }
//                    break;
//                case CONE_STACK1:
//                    if (!drive.isBusy()){
//                        drive.closeClaw();
//                        sleep(300);
//                        currentState= State.PICKUP;
//                        liftTarget=300;
//                    }
//                    break;
//                case PICKUP:
//                    drive.closeClaw();
//                    //liftTarget = 200;
//                    if (drive.linearSlide.getCurrentPosition()>250){
//                        currentState = State.SCORE_CONE;
//                        armTarget= ARM_BACK;
//                        liftTarget = SLIDE_HIGH;
//                        drive.followTrajectoryAsync(scoreNewCone);
//                    }
//                    break;
////                case BACK_UP:
////                    if (!drive.isBusy()){
////                        armTarget = ARM_BACK;
////                        drive.followTrajectory(scoreNewCone);
////                        currentState= State.SCORE_CONE;
////                    }
////                    break;
//
//
//                case SCORE_CONE:
//
//                    if (!drive.isBusy()) {
//                        drive.openClaw();
//                        sleep(500);
//                        currentState = State.PARK_GREEN;
//                    }
////                        time = new Date().getTime() - startTime;
////                        if (time< 25_000){
////                            //go get other cone
////                            currentState = State.NEW_CONE_FROM_SCORE_1;
////                            drive.followTrajectoryAsync(fromScoreNewConeToConeStack);
////
////                        } else {
////                            //go park
////                            if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GRAY) {
////                                drive.followTrajectoryAsync(parkGray);
////                                currentState = State.PARK_GRAY;
////                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {
////                                drive.followTrajectoryAsync(parkRed);
////                                currentState = State.PARK_GREEN;
////                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.RED) {
////                                currentState = State.PARK_RED;
////                            }
////                        }
////                    } else {
////                        if (drive.armMotor.getCurrentPosition()>100){
////                            liftTarget = SLIDE_HIGH;
////                        }
////                    }
//                    break;
////                case NEW_CONE:
////                    if (!drive.isBusy()) {
////                        currentState = State.SCORE_CONE;
////                        drive.followTrajectoryAsync(scoreNewCone);
////                    } else if(getRuntime() == 900000000) {
////
////                    }
////                    break;
//                case PARK_GRAY:
//                case PARK_RED:
//                case PARK_GREEN:
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
////                case PARK_RED:
////
////                    liftTarget= 0;
////                    if (drive.linearSlide.getCurrentPosition()<100){
////                        armTarget = 0;
////                    }
////                    if (!drive.isBusy()) {
////                        if (drive.armMotor.getCurrentPosition()<50){
////                            drive.openClaw();
////                            drive.tipCenter();
////                        }
////                    }
////                    break;
////                case PARK_GREEN:
////                    liftTarget= 0;
////                    if (drive.linearSlide.getCurrentPosition()<100){
////                        armTarget = 0;
////                    }
////                    if (drive.armMotor.getCurrentPosition()<50){
////                        drive.openClaw();
////                        drive.tipCenter();
////                    }
////                    break;
//                case DONE:
//                    break;
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

        }


    }

}
