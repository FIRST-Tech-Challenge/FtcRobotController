package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_CONE_STACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP_AUTO;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH_AUTO;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_MIDDLE;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_THROUGH;
import static org.firstinspires.ftc.masters.BadgerConstants.STACK_OFFSET;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.DriveConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;
import java.util.List;

@Config
@Autonomous(name = "Power Play Right Cycling World", group = "competition")
public class PowerPlayRightCyclingWORLD extends LinearOpMode {

    enum State {
        FIRST_DEPOSIT_PATH_1,
        FIRST_DEPOSIT_TURN,
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
        CYCLE_LOW,

        PICKUP,
        BACK_UP,
        ALIGN,

        PARK_GRAY,
        PARK_RED,
        PARK_GREEN,
        DONE,
        LIFT

    }

    int coneStack = ARM_CONE_STACK;
    SampleMecanumDrive drive;

    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    int armTarget = 0, liftTarget = 0;

    public static double xStack = 53;
    public static double yStack = -14;

    public static int turnJunction = 45;
    boolean retractArm=false;

    @Override
    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(36, -64.25), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        armPIDController = new ArmPIDController(drive.armMotor);

        drive.tipCenter();
        drive.closeAutoClaw();

        State currentState;

        // Trajectory from start to nearest tall pole
        Trajectory firstDepositPath1 = drive.trajectoryBuilder(startPose)
                //.splineToConstantHeading(new Vector2d(35, -14), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(new Vector2d(34, -45), Math.toRadians(90)))
                .splineTo(new Vector2d(34, -12 ), Math.toRadians(135))
//                .lineToSplineHeading(new Pose2d(new Vector2d(37, -30), Math.toRadians(90)))
//                .splineToSplineHeading(new Pose2d(36, -12 , Math.toRadians(135)), Math.toRadians(135))
                .build();

//        Trajectory firstDepositPath1 = drive.trajectoryBuilder(startPose)
//                .splineToConstantHeading(new Vector2d(36, -12), Math.toRadians(90))
//                .build();

//        Trajectory firstDepositScoreCone = drive.trajectoryBuilder(firstDepositPath1.end().plus(new Pose2d(0, 0, Math.toRadians(turnJunction))))
//                .lineTo(new Vector2d(31, -8))
//                .build();

        Trajectory backUpFromJunction = drive.trajectoryBuilder(firstDepositPath1.end())
                .back(8)
                .build();


        Trajectory cyclePickupPath1 = drive.trajectoryBuilder(backUpFromJunction.end().plus(new Pose2d(0,0,Math.toRadians(-turnJunction-90))))
                .splineToLinearHeading(new Pose2d(new Vector2d(xStack+9, yStack),Math.toRadians(0)),Math.toRadians(0))
                // .lineTo(new Vector2d(xStack, yStack))
                .build();


        Trajectory cycleScorePath1 = drive.trajectoryBuilder(cyclePickupPath1.end())
                .lineToSplineHeading(new Pose2d(new Vector2d(45, -14), Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(39, -12, Math.toRadians(135+180)), Math.toRadians(135))
                .build();

        Trajectory cycleDepositScoreCone = drive.trajectoryBuilder(cycleScorePath1.end().plus(new Pose2d(0, 0, Math.toRadians(-turnJunction))))
                .lineTo(new Vector2d(31, -7))
                .build();

        Trajectory cycleBackUpFromJunction = drive.trajectoryBuilder(cycleDepositScoreCone.end())
                .back(6)
                .build();
//
//        Trajectory fromScoreNewConeToConeStack = drive.trajectoryBuilder(cycleScorePath1.end())
//                .lineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(0)))
//                .build();


        waitForStart();

        drive.closeAutoClaw();
        drive.tipCenter();

        long startTime = new Date().getTime();
        long time = 0;

        long alignTime = new Date().getTime();

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

//            telemetry.addData("Position", sleeveColor);
//            telemetry.update();
        }
        CV.setPipeDetectionFront();

        currentState = State.FIRST_DEPOSIT_PATH_1;
        drive.followTrajectoryAsync(firstDepositPath1);
        armTarget = ARM_MID_TOP;
        int slidePosition = drive.linearSlide.getCurrentPosition();
        int armPosition = drive.armMotor.getCurrentPosition();
        drive.switchFollower();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentState) {
//                case FIRST_DEPOSIT_PATH_1:
//                    if (!drive.isBusy()) {
//
//                        currentState = PowerPlayRightCyclingWORLD.State.FIRST_DEPOSIT_TURN;
//                        drive.turnAsync(Math.toRadians(turnJunction));
//                    } else {
//                        armTarget = ARM_MID_TOP_AUTO;
//                        if (armPosition > 100) {
//                            liftTarget = SLIDE_HIGH_AUTO;
//                            drive.tipFront();
//                            drive.closeClaw();
//                        }
//                    }
//                    break;
//                case FIRST_DEPOSIT_TURN:
//                    if (!drive.isBusy()) {
//                        //drive.followTrajectoryAsync(firstDepositScoreCone);
//                        currentState = PowerPlayRightCyclingWORLD.State.ALIGN;
//                        alignTime = new Date().getTime();
//                    }
//                    break;

                case FIRST_DEPOSIT_PATH_1:
                    if (!drive.isBusy()) {

                        currentState = State.ALIGN;
                        alignTime = new Date().getTime();
                    } else {
                        armTarget = ARM_MID_TOP_AUTO;
                        if (armPosition > 100) {
                            liftTarget = SLIDE_HIGH_AUTO;
                            drive.tipFront();
                            drive.closeAutoClaw();
                        }
                    }
                    break;

                case ALIGN:
                    if (drive.alignPole(CV.sleevePipeline.position) || new Date().getTime()- alignTime >500){
                        telemetry.addData("done aligning", "score cone");
                        currentState = State.FIRST_DEPOSIT_SCORE_CONE;
                        Trajectory trajForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(5)
                                .build();
                        drive.followTrajectoryAsync(trajForward);
                    }
                    break;
                case FIRST_DEPOSIT_SCORE_CONE:
                    if (!drive.isBusy()) {
                        drive.openClaw();
                        sleep(250);
                        drive.closeAutoClaw();
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
                        if (armPosition<0){
                            retractArm = true;
                        } else {
                            setArmToConeStack(slidePosition);
                        }
                        drive.switchOriginalFollower();
                        currentState = State.CYCLE_PICKUP_PATH1;
                        drive.followTrajectoryAsync(cyclePickupPath1);

                    } else {

                        if (!retractArm) {
                            liftTarget = 0;
                            if (drive.linearSlide.getCurrentPosition() < 100) {
                                armTarget = coneStack;
                                drive.openClaw();
                                drive.tipCenter();
                            }
                        }
                    }
                    break;
                case CYCLE_PICKUP_PATH1:
                    if (!drive.isBusy()){

                        drive.closeAutoClaw();
                        sleep(250);
                        liftTarget= SLIDE_MIDDLE;
                        currentState = State.LIFT;
                    } else {
                        drive.openClaw();
                        drive.tipCenter();
                        armTarget = coneStack;
                    }
                    break;

                case LIFT:
                    if (slidePosition>SLIDE_MIDDLE-30){
                        drive.followTrajectoryAsync(cycleScorePath1);
                        currentState = State.CYCLE_SCORE_PATH1;

                        coneStack = coneStack-STACK_OFFSET;
                        drive.switchFollower();

                    }
                    break;
                case  CYCLE_SCORE_PATH1:
                    if (!drive.isBusy()){
                        currentState = State.CYCLE_SCORE_ALIGN;
                        alignTime = new Date().getTime();
//                        currentState = State.CYCLE_SCORE_TURN;
//                        drive.turnAsync(Math.toRadians(-45));7
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
//                        armTarget = ARM_BACK;
                    } else{
                        liftTarget= SLIDE_THROUGH;
                        if (drive.linearSlide.getCurrentPosition()>SLIDE_THROUGH-100){
                            armTarget= ARM_BACK;
                        }
                        if (drive.armMotor.getCurrentPosition()<-1100){
                            drive.tipBack();
                        }
                        //tipBack(armPosition);

                    }
                    break;

                case CYCLE_SCORE_ALIGN:
                    if (drive.alignPole(CV.pipeDetectionPipeline.position) || new Date().getTime()- alignTime >500){
                        currentState = State.CYCLE_SCORE_CONE;
                        cycleDepositScoreCone = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(9)
                                .build();
                        drive.followTrajectoryAsync(cycleDepositScoreCone);
                        drive.tipBack();

                    }
                    break;
                case CYCLE_SCORE_CONE:
                    if (!drive.isBusy()) {

                        //sleep(100);
                        drive.openClaw();
                        cycleBackUpFromJunction=drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(8)
                                .build();
                        sleep(250);
                        drive.followTrajectoryAsync(cycleBackUpFromJunction);
                        currentState = State.CYCLE_BACK_UP;
                    }
                    tipBack(armPosition);
                    break;
                case CYCLE_BACK_UP:
                    if (!drive.isBusy()){
                        retractArm= true;
                        time = new Date().getTime() - startTime;
                        if (time> 22*1000) {

                            currentState = State.CYCLE_PICKUP_END;
                            drive.switchOriginalFollower();
                            drive.turnAsync(Math.toRadians(-turnJunction));
                        } else {

                            currentState = State.CYCLE_PICKUP_TURN;
                            drive.turnAsync(Math.toRadians(turnJunction));

                        }

                    } else {
                        drive.closeAutoClaw();
                    }
                    break;

                case CYCLE_PICKUP_END:

                    if (!drive.isBusy()){
                        retractArm = true;

                        switch (sleeveColor){
                            case GREEN:
                                Trajectory parkGreen1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(new Vector2d(63, -14), Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(60))
                                        .build();
                                drive.followTrajectoryAsync(parkGreen1);
                                currentState = State.PARK_GREEN;
                                break;
                            case RED:
                                Trajectory parkRed = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(new Vector2d(36, -24),Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(60))
                                        .build();
                                drive.followTrajectoryAsync(parkRed);
                                currentState = State.PARK_RED;
                                break;
                            case GRAY:
                                Trajectory parkGray1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d( new Vector2d(12, -14), Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(60))
                                        .build();
                                drive.followTrajectoryAsync(parkGray1);
                                currentState = State.PARK_GRAY;
                                break;
                        }
                    }
                    break;



                case PARK_GREEN:

                    if (!retractArm) {
                        liftTarget = 0;
                        if (drive.linearSlide.getCurrentPosition() < 100) {
                            armTarget = coneStack;
                            drive.tipCenter();
                        }
                        if (!drive.isBusy()) {
                            if (drive.armMotor.getCurrentPosition() < 50) {
                                drive.openClaw();
                                drive.tipCenter();
                            }
                            currentState = State.DONE;
                        }
                    }

                case PARK_RED:
                    if (!retractArm) {
                        liftTarget = 0;
                        if (drive.linearSlide.getCurrentPosition() < 100) {
                            armTarget = coneStack;
                            drive.tipCenter();
                        }
                        if (!drive.isBusy()) {
                            if (drive.armMotor.getCurrentPosition() < 50) {
                                drive.openClaw();
                                drive.tipCenter();
                            }
                            currentState = State.DONE;
                        }
                    }

                    break;

                case PARK_GRAY:
                    if (!retractArm) {
                        liftTarget = 0;
                        if (drive.linearSlide.getCurrentPosition() < 100) {
                            armTarget = coneStack;
                            drive.tipCenter();
                        }
                        if (!drive.isBusy()) {
                            Trajectory parkGray2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d( new Vector2d(12, -24), Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(60))
                                    .build();
                            drive.followTrajectoryAsync(parkGray2);
                            if (drive.armMotor.getCurrentPosition() < 50) {
                                drive.openClaw();
                                drive.tipCenter();
                            }
                            currentState = State.DONE;
                        }
                    }


                    break;
                case DONE:
                    if (!drive.isBusy()) {
                        if (drive.linearSlide.getCurrentPosition() < 100) {
                            armTarget = 0;
                            drive.tipCenter();
                        }
                        if (armPosition < 50) {
                            drive.openClaw();
                            drive.tipCenter();
                        }

                    }
                    drive.tipCenter();
                    break;
            }

            if (retractArm){
                armTarget = coneStack;
                if (drive.armMotor.getCurrentPosition()>100){
                    liftTarget=0;
                    retractArm = false;
                    drive.tipCenter();
                }
            }


            armPosition = drive.armMotor.getCurrentPosition();

            armPIDController.setTarget(armTarget);
            drive.armMotor.setPower(armPIDController.calculateVelocity(armPosition));
            // }

            liftPIDController.setTarget(liftTarget);
            slidePosition = drive.linearSlide.getCurrentPosition();
            double power = liftPIDController.calculatePower(slidePosition);
            drive.linearSlide.setPower(power);
            drive.slideOtherer.setPower(liftPIDController.calculatePowerSingle(drive.slideOtherer.getCurrentPosition()));
            //drive.slideOtherer.setPower(power);
            drive.frontSlide.setPower(power);


            //  telemetry.addData("power ", power);
            telemetry.addData("arm target", armTarget);
            telemetry.addData("arm position", armPosition);
            telemetry.addData("lift target", liftTarget);
            telemetry.addData("lift position", slidePosition);

            //  telemetry.update();

        }
    }

    protected void setArmToConeStack(int slidePosition){
        liftTarget = 0;
        armTarget = coneStack;
        if (slidePosition < 100) {
            drive.openClaw();
            drive.tipCenter();
        }
    }

    protected void tipBack(int armPosition){
        if (armPosition <- 900) {
            drive.tipBack();
        }
    }

}
