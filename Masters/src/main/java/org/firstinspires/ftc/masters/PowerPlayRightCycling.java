package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_CONE_STACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_MIDDLE;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_THROUGH;

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
        FIRST_DEPOSIT_PATH_1,
        FIRST_DEPOSIT_TURN,
        FIRST_DEPOSIT_SCORE_CONE,

        BACK_UP_FROM_JUNCTION,

        CYCLE_PICKUP_TURN,
        CYCLE_PICKUP_PATH1,
        CYCLE_PICKUP_PATH2,

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
    boolean retractArm=false;

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
        Trajectory firstDepositPath1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(36, -14), Math.toRadians(90))
                .build();

        Trajectory firstDepositScoreCone = drive.trajectoryBuilder(firstDepositPath1.end().plus(new Pose2d(0, 0, Math.toRadians(turnJunction))))
                .lineTo(new Vector2d(31, -8))
                .build();

        Trajectory backUpFromJunction = drive.trajectoryBuilder(firstDepositScoreCone.end())
                .back(6)
                .build();


        Trajectory cyclePickupPath1 = drive.trajectoryBuilder(backUpFromJunction.end().plus(new Pose2d(0,0,Math.toRadians(-turnJunction-90))))
                .splineToLinearHeading(new Pose2d(new Vector2d(53, -13),0),0)
               // .lineTo(new Vector2d(xStack, yStack))
                .build();
        Trajectory cyclePickupPath2 = drive.trajectoryBuilder(cyclePickupPath1.end())
                .lineTo(new Vector2d(xStack+7.5, yStack))
                .build();
        ;

        Trajectory cycleScorePath1 = drive.trajectoryBuilder(cyclePickupPath2.end())
                .lineTo(new Vector2d(36, -13))
                .build();

        Trajectory cycleDepositScoreCone = drive.trajectoryBuilder(cycleScorePath1.end().plus(new Pose2d(0, 0, Math.toRadians(-turnJunction))))
                .lineTo(new Vector2d(31, -8))
                .build();

        Trajectory cycleBackUpFromJunction = drive.trajectoryBuilder(cycleDepositScoreCone.end())
                .back(6)
                .build();

        Trajectory fromScoreNewConeToConeStack = drive.trajectoryBuilder(cycleScorePath1.end())
                .lineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(0)))
                .build();

        Trajectory parkGray = drive.trajectoryBuilder(cycleBackUpFromJunction.end().minus(new Pose2d(0, 0, Math.toRadians(turnJunction))))
                .lineTo(new Vector2d(12, -12))

                // .splineToLinearHeading(new Pose2d(new Vector2d(11.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
                .build();

        Trajectory parkGreen = drive.trajectoryBuilder(cycleBackUpFromJunction.end().minus(new Pose2d(0, 0, Math.toRadians(turnJunction))))
                .lineTo(new Vector2d(60, -12))
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

        long alignTime = new Date().getTime();

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }
        CV.setPipeDetectionFront();

        currentState = State.FIRST_DEPOSIT_PATH_1;
        drive.followTrajectoryAsync(firstDepositPath1);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
                case FIRST_DEPOSIT_PATH_1:
                    if (!drive.isBusy()) {

                        currentState = State.FIRST_DEPOSIT_TURN;
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
                case FIRST_DEPOSIT_TURN:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(firstDepositScoreCone);
                        currentState = State.ALIGN;
                        alignTime = new Date().getTime();
                    }
                    break;
                case ALIGN:
                    if (drive.alignPole(CV.sleevePipeline.position) || new Date().getTime()- alignTime >1000){
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

                        //sleep(100);
                        drive.openClaw();
                        sleep(500);
                        drive.closeClaw();
                        drive.followTrajectoryAsync(backUpFromJunction);
                        currentState = State.BACK_UP_FROM_JUNCTION;
                        CV.sleeveWebcam.stopStreaming();
                    }
                    break;
                case BACK_UP_FROM_JUNCTION:
                    if (!drive.isBusy()) {
                        currentState = State.CYCLE_PICKUP_TURN;
                        drive.turnAsync(Math.toRadians(-turnJunction-90));
                    }
                    break;
                case CYCLE_PICKUP_TURN:
                    if (!drive.isBusy()){
                        time = new Date().getTime() - startTime;
                        if (time>22*1000){
                            if (drive.linearSlide.getCurrentPosition() < 100) {
                                armTarget = 0;
                            }
                            if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.RED) {

                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GREEN) {
                                drive.followTrajectoryAsync(parkGreen);
                                currentState = State.PARK_GREEN;
                            } else if (sleeveColor == PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor.GRAY) {
                                currentState = State.PARK_GRAY;
                                drive.followTrajectoryAsync(parkGray);
                            }
                            //time to go park
                        } else {
                            currentState = State.CYCLE_PICKUP_PATH1;
                            drive.followTrajectoryAsync(cyclePickupPath1);
                        }
                    } else {
                        if (!retractArm) {
                            liftTarget = 0;
                            if (drive.linearSlide.getCurrentPosition() < 100) {
                                armTarget = ARM_CONE_STACK;
                            }
                        }
                    }
                    break;
                case CYCLE_PICKUP_PATH1:
                    if (!drive.isBusy()){
                        armTarget = ARM_CONE_STACK-70;
                        drive.openClaw();
                        drive.tipCenter();
                        drive.followTrajectoryAsync(cyclePickupPath2);
                        currentState= State.CYCLE_PICKUP_PATH2;
                    }
                    break;
                case CYCLE_PICKUP_PATH2:
                    if (!drive.isBusy()){
                        drive.closeClaw();
                        sleep(300);
                        liftTarget= SLIDE_MIDDLE;
                        currentState = State.LIFT;
                    }
                    break;
                case LIFT:
                    if (drive.linearSlide.getCurrentPosition()>SLIDE_MIDDLE-30){
                        drive.followTrajectoryAsync(cycleScorePath1);
                        currentState= State.CYCLE_SCORE_PATH1;

                    }
                    break;
                case  CYCLE_SCORE_PATH1:
                    if (!drive.isBusy()){
                        currentState = State.CYCLE_SCORE_TURN;
                        drive.turnAsync(Math.toRadians(-45));
                    } else {
                        liftTarget= SLIDE_THROUGH;
                        if (drive.linearSlide.getCurrentPosition()>SLIDE_THROUGH-100){
                            armTarget= ARM_BACK;
                        }
                        if (drive.armMotor.getCurrentPosition()<-1100){
                            drive.tipBack();
                        }
                    }
                    break;
                case CYCLE_SCORE_TURN:
                    if (!drive.isBusy()){
                        currentState = State.CYCLE_SCORE_ALIGN;
                        alignTime = new Date().getTime();
                        //drive.followTrajectoryAsync(cycleDepositScoreCone);
                    } else{
                        liftTarget= SLIDE_THROUGH;
                        if (drive.linearSlide.getCurrentPosition()>SLIDE_THROUGH-100){
                            armTarget= ARM_BACK;
                        }
                        if (drive.armMotor.getCurrentPosition()<-1100){
                            drive.tipBack();
                        }

                    }
                    break;

                case CYCLE_SCORE_ALIGN:
                    if (drive.alignPole(CV.pipeDetectionPipeline.position) || new Date().getTime()- alignTime >1000){
                        currentState = State.CYCLE_SCORE_CONE;
                        cycleDepositScoreCone = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(8)
                                .build();
                        drive.followTrajectoryAsync(cycleDepositScoreCone);
                    }
                    break;
                case CYCLE_SCORE_CONE:
                    if (!drive.isBusy()) {
                        cycleBackUpFromJunction=drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(8)
                                .build();
                        //sleep(100);
                        drive.openClaw();
                        sleep(500);


                        drive.followTrajectoryAsync(cycleBackUpFromJunction);
                        currentState = State.CYCLE_BACK_UP;

                    }
                    break;
                case CYCLE_BACK_UP:
                    if (!drive.isBusy()){
                        retractArm= true;
                        currentState = State.CYCLE_PICKUP_TURN;
                        drive.turnAsync(Math.toRadians(-45));

                    } else {
                        drive.closeClaw();
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
                case PARK_GRAY:
                case PARK_RED:
                case PARK_GREEN:

                    if (!retractArm) {
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
                    }
                    break;
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

            if (retractArm){
                armTarget = ARM_CONE_STACK;
                if (drive.armMotor.getCurrentPosition()>100){
                    liftTarget=0;
                    retractArm = false;
                    drive.tipCenter();
                }
            }


            armPIDController.setTarget(armTarget);
            double armPower = armPIDController.calculateVelocity();
            if (armTarget==ARM_BACK){
                if (armPower<0){
                    armPower= Math.max(armPower, -0.5);
                }
                else armPower = Math.min(armPower, 0.5);
            }
            drive.armMotor.setPower(armPower);

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

          //  telemetry.update();

        }


    }

}
