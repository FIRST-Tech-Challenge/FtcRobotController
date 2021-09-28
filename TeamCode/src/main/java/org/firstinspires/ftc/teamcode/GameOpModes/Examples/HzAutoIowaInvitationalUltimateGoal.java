package org.firstinspires.ftc.teamcode.GameOpModes.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzArmUltimateGoal;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzAutonomousControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzGamepadControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzIntakeUltimateGoal;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzLaunchSubControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzLauncherUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzMagazineUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzVuforiaUltimateGoal;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;


/**
 * Ultimate Goal Autonomous mode <BR>
 *
 * This code describes how Autonomous mode is done by Hazmat Robot for Ultimate Goal.<BR>
 * The following options are coded here, and selectable through Gamepad inputs to set up <BR>
 *     <emsp>Playing Alliance : Red or Blue</emsp>
 *     <emsp>Start Line : Inner or Outer</emsp>
 *     <emsp>Game options :</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch, move Wobble Goal2 and park</emsp>
 *
 * The code for Red and Blue are written as reflection of each other.<BR>
 * Camera on either side is used using Vuforia to determine target for Wobble Goal<BR>
 */
@Autonomous(name = "Hazmat Iowa State Championship Ultimate Goal", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp RR Ultimate Goal")
@Disabled
public class HzAutoIowaInvitationalUltimateGoal extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepadControllerUltimateGoal hzGamepadControllerUltimateGoal;
    public HzAutonomousControllerUltimateGoal hzAutonomousController;
    public HzDrive hzDrive;
    public HzMagazineUltimateGoal hzMagazineUltimateGoal;
    public HzIntakeUltimateGoal hzIntakeUltimateGoal;
    public HzLaunchSubControllerUltimateGoal hzLaunchSubControllerUltimateGoal;
    public HzLauncherUltimateGoal hzLauncherUltimateGoal;
    public HzArmUltimateGoal hzArmUltimateGoal;

    public HzVuforiaUltimateGoal hzVuforiaUltimateGoal;
    public Pose2d startPose = HzGameFieldUltimateGoal.BLUE_INNER_START_LINE;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public HzGameFieldUltimateGoal.TARGET_ZONE targetZone = HzGameFieldUltimateGoal.TARGET_ZONE.A;
    public HzVuforiaUltimateGoal.ACTIVE_WEBCAM activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;

    double af = HzGameFieldUltimateGoal.ALLIANCE_FACTOR;

    double turnAnglePowershot12 = Math.toRadians(-5);
    double turnAnglePowershot23 = Math.toRadians(-5);
    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize HW
        hzDrive = new HzDrive(hardwareMap);
        hzMagazineUltimateGoal = new HzMagazineUltimateGoal(hardwareMap);
        hzIntakeUltimateGoal = new HzIntakeUltimateGoal(hardwareMap);

        hzLauncherUltimateGoal = new HzLauncherUltimateGoal(hardwareMap);
        hzArmUltimateGoal = new HzArmUltimateGoal(hardwareMap);
        hzLaunchSubControllerUltimateGoal = new HzLaunchSubControllerUltimateGoal(hardwareMap, hzLauncherUltimateGoal, hzIntakeUltimateGoal, hzMagazineUltimateGoal, hzDrive);
        hzGamepadControllerUltimateGoal = new HzGamepadControllerUltimateGoal(gamepad1,hzDrive, hzMagazineUltimateGoal, hzIntakeUltimateGoal, hzLaunchSubControllerUltimateGoal, hzLauncherUltimateGoal, hzArmUltimateGoal);
        hzAutonomousController = new HzAutonomousControllerUltimateGoal(hzDrive, hzMagazineUltimateGoal, hzIntakeUltimateGoal, hzLaunchSubControllerUltimateGoal, hzLauncherUltimateGoal, hzArmUltimateGoal);

        //Key Pay inputs to select Game Plan;
        //selectGamePlan();
        iowaGamePlan();
        hzVuforiaUltimateGoal = new HzVuforiaUltimateGoal(hardwareMap, activeWebcam);
        af = HzGameFieldUltimateGoal.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        hzVuforiaUltimateGoal.activateVuforiaTensorFlow();

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        hzIntakeUltimateGoal.setIntakeReleaseHold();
        hzAutonomousController.setMagazineToLaunch();

        hzLauncherUltimateGoal.flyWheelVelocityHighGoal = 1430;
        hzLauncherUltimateGoal.flyWheelVelocityPowerShot = 1330;

        while (hzMagazineUltimateGoal.magazineLaunchTouchSensor.isPressed() == false) {
            hzAutonomousController.setMagazineToLaunch();
            if (isStopRequested()) return;
        }

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow
            targetZone = hzVuforiaUltimateGoal.runVuforiaTensorFlow();

            if (!parked){
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.runAutoControl();
            }

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
            hzLaunchSubControllerUltimateGoal.launchMode = HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.MANUAL;

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {

                hzVuforiaUltimateGoal.deactivateVuforiaTensorFlow();
                runAutoInner();

                hzIntakeUltimateGoal.setIntakeReleaseOpen();
                hzAutonomousController.setMagazineToCollect();

                //Move to Launching Position
                parked = true;

                //Write last position to static class to be used as initial position in TeleOp
                HzGameFieldUltimateGoal.currentPose = hzDrive.getPoseEstimate();
                HzGameFieldUltimateGoal.poseSetInAutonomous = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        //Write last position to static class to be used as initial position in TeleOp
        HzGameFieldUltimateGoal.currentPose = hzDrive.getPoseEstimate();
        HzGameFieldUltimateGoal.poseSetInAutonomous = true;
    }

    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoInner(){

        // Move to launch position and launch rings to High Goal or Powershots
        if (!hzAutonomousController.launchHighGoalOrPowerShot) {
            //runInnerOnlyLaunchPark(0);
            return;
        } else {
            // Move to launch position and launch rings to High Goal or Powershots
            if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
                // Set magazine to Launch in case it slipped
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();

                hzAutonomousController.setMagazineToLaunch();
                //Move to position to launch rings
                if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-6, 14, Math.toRadians(19)))//-10
                            .build();
                }
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();
            } else {
                hzAutonomousController.setLaunchTargetPowerShot1();

                //Intermediary position to move away from alliance robot.
                //For option of second wobble goal drop, this is avoided to save time
                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                            .build();
                    hzDrive.followTrajectory(traj);
                }
                //Move to position to launch rings

                if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, af * 15, Math.toRadians(af * 5)))
                            .lineToLinearHeading(new Pose2d(-10, af * 19, Math.toRadians(af * 5)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    //Set turn angles prior to launching for each of the power shorts
                    turnAnglePowershot12 = Math.toRadians(af * -5);
                    turnAnglePowershot23 = Math.toRadians(af * -7);
                } else {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, af * 15, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    //Set turn angles prior to launching for each of the power shorts
                    turnAnglePowershot12 = Math.toRadians(af * -5);
                    turnAnglePowershot23 = Math.toRadians(af * -7);
                }


                launch3RingsToPowerShots();
            }
        }

        // For case when wobble goal is not to be done and only park.

        if (!hzAutonomousController.dropFirstWobbleGoal) {
            //runInnerOnlyLaunchPark(0);
            return;
        } else {

            // Move to drop wobble goal on target
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                switch (targetZone) {
                    case A:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(0, af * 55, Math.toRadians(af * -135)))//43
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                 .lineToSplineHeading(new Pose2d(25, af * 33, Math.toRadians(af * -135)))
                                 .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case C:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(49, af * 61, Math.toRadians(af * -135))) //y:51 STATE y:43
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                }
            }

            dropWobbleGoalInTarget();

            if ((hzAutonomousController.pickRingFromTargetMarker == false) || (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A)) { // Park
                if (hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    hzWait(300);
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-8, af * 37, Math.toRadians(5))) //STATE y: 31
                            .build();
                    hzDrive.followTrajectory(traj);
                    runInnerPickAndDropSecondWobbleGoalAndPark();
                }
                return;
            } else { //Target Zone is B or C and pickRingFromTargetMarker == True
                //Pick rings from Target Marker
                runPickRingsFromTargetMarker();

                if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal) {
                    hzAutonomousController.setIntakeStop();
                    hzAutonomousController.setMagazineToLaunch();
                    hzAutonomousController.setLaunchTargetHighGoal();

                    if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-8, af * 37, Math.toRadians(5))) //STATE y: 31
                                .build();
                        hzDrive.followTrajectory(traj);
                    } else {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-8, af * 34, Math.toRadians(0))) //STATE x :-13angle:5
                                .build();
                        hzDrive.followTrajectory(traj);
                    }

                    launch3RingsToHighGoal();

                    if (hzAutonomousController.pickAndDropSecondWobbleGoal == true) {
                        runInnerPickAndDropSecondWobbleGoalAndPark();
                    } else {
                        //Park
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13, af * 36, Math.toRadians(af * 0)))
                                .build();
                        hzDrive.followTrajectory(traj);
                    }
                } else { //Move to baseline and park in safe zone
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-50, af * 16, Math.toRadians(af * -90)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(10, af * 0, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    hzAutonomousController.setIntakeStop();
                }
            }
        }
    }

    /**
     * Path for picking and dropping second wobble goal for Inner Start position
     */
    public void runInnerPickAndDropSecondWobbleGoalAndPark() {
        hzAutonomousController.setMoveArmPickWobble();
        hzAutonomousController.runOpenGrip();
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-20, af * 48), Math.toRadians(af * 0))
                .splineToConstantHeading(new Vector2d(-33, af * 48), Math.toRadians(af * 0))
                .build();
        hzDrive.followTrajectory(traj);
        hzWait(200);
        hzAutonomousController.runCloseGrip();
        hzWait(400);
        hzAutonomousController.setMoveArmHoldUpWobbleRing();
        hzWait(500);
        switch (targetZone) {
            case A:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(6, af * 42, Math.toRadians(af * -90)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af * 42, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(42, af * 61, Math.toRadians(af * -135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
        }
        dropWobbleGoalInTarget();
        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(20, af * 56, Math.toRadians(af * -135)))
                    .build();
            hzDrive.followTrajectory(traj);
        } else {
            if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.B) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(0, af * 27, Math.toRadians(af * 180)))
                    .build();
                hzDrive.followTrajectory(traj);
            }

            //Park
            //if (targetZone == HzGameField.TARGET_ZONE.C) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(3, af * 27, Math.toRadians(af * 0)))
                .build();
            hzDrive.followTrajectory(traj);
        //}
        }

    }


    public void runPickRingsFromTargetMarker(){
        hzIntakeUltimateGoal.setIntakeReleaseOpen();
        hzAutonomousController.setIntakeStart();
        hzWait(500);

        //Move to Position to pick rings
        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.B) {

            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 36.5, Math.toRadians(af * 180)))
                        // .lineToLinearHeading(new Pose2d(0, af * 33.5, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22, af * 36.5, Math.toRadians(af * 180)))
                        // .lineToLinearHeading(new Pose2d(-22, af * 33.5, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
            } else {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 35.5, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22, af * 35.5, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
            }
        }

        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 36.5, Math.toRadians(af * -180)))
                        // .lineToLinearHeading(new Pose2d(0, af * 33.5, Math.toRadians(af * -180)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22, af * 36.5, Math.toRadians(af * -180)))
                        // .lineToLinearHeading(new Pose2d(-22, af * 33.5, Math.toRadians(af * -180)))
                        .build();
                hzDrive.followTrajectory(traj);

                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-22, af * 35.5, Math.toRadians(af * -180)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }
            } else {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 35.5, Math.toRadians(af * -180)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-11, af * 35.5, Math.toRadians(af * -180)))
                        .build();
                hzDrive.followTrajectory(traj);

                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * -180)))
                            .lineToLinearHeading(new Pose2d(-22, af * 35.5, Math.toRadians(af * -180)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }
            }
        }
        hzWait(300);
    }


    /**
     * Sequence of launching 3 rings to High Goal with time in between
     */
    public void launch3RingsToHighGoal1(){
        hzAutonomousController.setLaunchTargetHighGoal();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(200);
        hzAutonomousController.setLaunchTargetHighGoal();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(450); //400
        hzAutonomousController.setRunLauncherTrue();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(450);
        hzAutonomousController.setRunLauncherTrue();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(450);
        hzAutonomousController.setRunLauncherTrue();

        // Run 4th just in case
        if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
            hzAutonomousController.setMagazineToLaunch();
            hzWait(450);
            hzAutonomousController.setRunLauncherTrue();
        }
        hzWait(600);

        hzAutonomousController.setLaunchTargetOff();
        hzAutonomousController.setMagazineToCollect();
    }

    /**
     * Sequence of launching 3 rings to High Goal
     */
    public void launch3RingsToHighGoal(){
        hzAutonomousController.setLaunchTargetHighGoal();
        hzAutonomousController.setMagazineToLaunch();
        //hzWait(1000); //400
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();
        hzAutonomousController.setLaunchTargetHighGoal();
        hzAutonomousController.setRunLauncherTrue();
        //hzWait(200);
        hzAutonomousController.setLaunchTargetHighGoal();
        hzAutonomousController.setRunLauncherTrue();

        // Run 4th just in case
        /*if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
            hzAutoControl.setLaunchTargetHighGoal();
            hzAutoControl.setRunLauncherTrue();
            hzAutoControl.setLaunchTargetHighGoal();
            hzAutoControl.setRunLauncherTrue();
        }*/
        hzWait(400);

        hzAutonomousController.setLaunchTargetOff();
        hzAutonomousController.setMagazineToCollect();
    }

    /**
     * Sequence of launching 3 rings to PowerShots
     */
    public void launch3RingsToPowerShots(){
        hzAutonomousController.setLaunchTargetPowerShot1();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(600);
        hzAutonomousController.setRunLauncherTrue();

        hzDrive.turn(turnAnglePowershot12);
        hzAutonomousController.setLaunchTargetPowerShot1();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();

        hzDrive.turn(turnAnglePowershot23);
        hzAutonomousController.setMagazineToLaunch();
        hzAutonomousController.setLaunchTargetPowerShot1();
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();
        hzWait(400);

        hzAutonomousController.setLaunchTargetOff();
        hzAutonomousController.setMagazineToCollect();

    }

    /**
     * Sequence of dropping Wobble Goal once it reaches target position
     */
    public void dropWobbleGoalInTarget(){
        hzAutonomousController.setMoveArmDropWobbleAutonoumous();
        //hzWait(1000);
        hzWait(800);
        hzAutonomousController.runOpenGrip();
        hzWait(700);
        //hzAutoControl.setMoveArmHoldUpWobbleRing();
        hzAutonomousController.setMoveArmParked();
    }


    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void hzWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            hzAutonomousController.runAutoControl();
        }
    }

    public void iowaGamePlan(){
        HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE;
        HzGameFieldUltimateGoal.ALLIANCE_FACTOR = 1;
        HzGameFieldUltimateGoal.startPosition = HzGameFieldUltimateGoal.START_POSITION.INNER;
        startPose = HzGameFieldUltimateGoal.BLUE_INNER_START_LINE;
        activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
        hzAutonomousController.launchHighGoalOrPowerShot = true;
        hzAutonomousController.autoLaunchAim = HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL;
        hzAutonomousController.dropFirstWobbleGoal = true;
        hzAutonomousController.pickRingFromTargetMarker = true;
        hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = true;
        hzAutonomousController.pickAndDropSecondWobbleGoal = true;
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", HzGameFieldUltimateGoal.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", HzGameFieldUltimateGoal.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", HzGameFieldUltimateGoal.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", hzDrive.driveMode);
        telemetry.addData("PoseEstimate :", hzDrive.poseEstimate);
        telemetry.addData("Battery Power", hzDrive.getBatteryVoltage(hardwareMap));

        //telemetry.addData("Visible Target : ", hzVuforia1.visibleTargetName);
        // Print pose to telemetry
        //telemetry.addData("PoseVuforia :",hzVuforia1.poseVuforia);

        telemetry.addData("targetZone identified: ", targetZone);

        //******* Magazine Debug ********
        switch (hzMagazineUltimateGoal.getMagazinePosition()) {
            case AT_LAUNCH: {
                telemetry.addData("hzMagazine.getMagazinePosition(): ", "MAGAZINE_AT_LAUNCH");
                break;
            }
            case AT_COLLECT: {
                telemetry.addData("hzMagazine.getMagazinePosition():", "MAGAZINE_AT_COLLECT");
                break;
            }
            case AT_ERROR: {
                telemetry.addData("hzMagazine.getMagazinePosition():", "MAGAZINE_AT_ERROR");
                break;
            }
        }
        //telemetry.addData("hzMagazine.moveMagazineToLaunchState",hzMagazine.moveMagazineToLaunchState);
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazineUltimateGoal.magazineLaunchTouchSensor.isPressed());
        //telemetry.addData("hzMagazine.moveMagazineToCollectState",hzMagazine.moveMagazineToCollectState);
        telemetry.addData("magazineCollectTouchSensor.getState():", hzMagazineUltimateGoal.magazineCollectTouchSensor.isPressed());


        //********** Intake Debug *******
        switch (hzIntakeUltimateGoal.getIntakeState()){
            case RUNNING: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_RUNNING");
                break;
            }
            case STOPPED: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_STOPPED");
                break;
            }
            case REVERSING: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_REVERSING");
                break;
            }
        }

        //******* Launch Controller Debug ********
        telemetry.addData("hzLaunchController.launchMode : ", hzLaunchSubControllerUltimateGoal.launchMode);
        telemetry.addData("hzLaunchController.deactivateLaunchReadinessState :", hzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState);
        telemetry.addData("hzLaunchController.activateLaunchReadinessState :", hzLaunchSubControllerUltimateGoal.activateLaunchReadinessState);
        telemetry.addData("hzLaunchController.launchReadiness : ", hzLaunchSubControllerUltimateGoal.launchReadiness);
        telemetry.addData("hzLaunchController.launchActivation : ", hzLaunchSubControllerUltimateGoal.launchActivation);
        telemetry.addData("hzLaunchController.lcTarget : ", hzLaunchSubControllerUltimateGoal.lcTarget);
        telemetry.addData("hzLaunchController.lcTargetVector", hzLaunchSubControllerUltimateGoal.lcTargetVector);
        telemetry.addData("hzLaunchController.distanceFromTarget : ", hzLaunchSubControllerUltimateGoal.distanceFromTarget);
        telemetry.addData("hzLauncher.launcherFlyWheelMotor.getPower() : ", hzLauncherUltimateGoal.launcherFlyWheelMotor.getPower());
        telemetry.addData("hzLaunchController.lclaunchMotorVelocity : ", hzLaunchSubControllerUltimateGoal.lclaunchMotorVelocity);
        telemetry.addData("hzLauncher.flyWheelVelocityPowerShot : ", hzLauncherUltimateGoal.flyWheelVelocityPowerShot);
        telemetry.addData("hzLauncher.flyWheelVelocityHighGoal : ", hzLauncherUltimateGoal.flyWheelVelocityHighGoal);
        telemetry.addData("hzLauncher.launcherFlyWheelMotor.getVelocity() : ", hzLauncherUltimateGoal.launcherFlyWheelMotor.getVelocity());
        telemetry.addData("hzDrive.drivePointToAlign : ", hzDrive.drivePointToAlign);

        //******* Launcher Debug *********
        //telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        telemetry.addData("launcherRingPlungerServo.getPosition() : ", hzLauncherUltimateGoal.launcherRingPlungerServo.getPosition());

        switch (hzLauncherUltimateGoal.getLauncherState()){
            case RUNNING_FOR_TARGET:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_TARGET");
                break;
            }
            case RUNNING_FOR_SUPPLY:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_SUPPLY");
                break;
            }
            case STOPPED:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_STOPPED");
                break;
            }
        }




        //***** Arm Debug ****
        telemetry.addData("armMotor.baseline", hzArmUltimateGoal.baselineEncoderCount);
        telemetry.addData("armMotor.getTargetPosition()", hzArmUltimateGoal.armMotor.getTargetPosition());
        telemetry.addData("armMotor.getCurrentPosition()", hzArmUltimateGoal.armMotor.getCurrentPosition());

        switch (hzArmUltimateGoal.getGripServoState()){
            case OPENED  : {
                telemetry.addData("hzArm.getGripServoState()", "OPENED");
                break;
            }
            case CLOSED: {
                telemetry.addData("hzArm.getGripServoState()", "CLOSED");
                break;
            }
        }

        switch (hzArmUltimateGoal.getCurrentArmPosition()){
            case PARKED: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "PARKED");
                break;
            }
            case DROP_WOBBLE_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "DROP_WOBBLE_RING");
                break;
            }
            case HOLD_UP_WOBBLE_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "HOLD_UP_WOBBLE_RING");
                break;
            }
            case PICK_WOBBLE:{
                telemetry.addData("hzArm.getCurrentArmPosition()", "PICK_WOBBLE");
                break;
            }
            case PICK_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "PICK_RING");
                break;
            }
        }

        telemetry.update();

    }
}

