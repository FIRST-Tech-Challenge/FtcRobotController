package org.firstinspires.ftc.teamcode.GameOpModes.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzArmUltimateGoal;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzAutonomousControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
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
@Autonomous(name = "Hazmat State Championship Ultimate Goal", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp RR Ultimate Goal")
@Disabled
public class HzAutoStateChampionshipUltimateGoal extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepadControllerUltimateGoal hzGamepadControllerUltimateGoal;
    public HzAutonomousControllerUltimateGoal hzAutonomousController;
    public DriveTrain driveTrain;
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
        driveTrain = new DriveTrain(hardwareMap);
        hzMagazineUltimateGoal = new HzMagazineUltimateGoal(hardwareMap);
        hzIntakeUltimateGoal = new HzIntakeUltimateGoal(hardwareMap);

        hzLauncherUltimateGoal = new HzLauncherUltimateGoal(hardwareMap);
        hzArmUltimateGoal = new HzArmUltimateGoal(hardwareMap);
        hzLaunchSubControllerUltimateGoal = new HzLaunchSubControllerUltimateGoal(hardwareMap, hzLauncherUltimateGoal, hzIntakeUltimateGoal, hzMagazineUltimateGoal, driveTrain);
        hzGamepadControllerUltimateGoal = new HzGamepadControllerUltimateGoal(gamepad1, driveTrain, hzMagazineUltimateGoal, hzIntakeUltimateGoal, hzLaunchSubControllerUltimateGoal, hzLauncherUltimateGoal, hzArmUltimateGoal);
        hzAutonomousController = new HzAutonomousControllerUltimateGoal(driveTrain, hzMagazineUltimateGoal, hzIntakeUltimateGoal, hzLaunchSubControllerUltimateGoal, hzLauncherUltimateGoal, hzArmUltimateGoal);

        //Key Pay inputs to select Game Plan;
        selectGamePlan();
        hzVuforiaUltimateGoal = new HzVuforiaUltimateGoal(hardwareMap, activeWebcam);
        af = HzGameFieldUltimateGoal.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        hzVuforiaUltimateGoal.activateVuforiaTensorFlow();

        driveTrain.getLocalizer().setPoseEstimate(startPose);

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

                if (HzGameFieldUltimateGoal.startPosition == HzGameFieldUltimateGoal.START_POSITION.INNER) {
                    runAutoInner();
                } else { //HzGameField.startPosition == HzGameField.START_POSITION.OUTER
                    runAutoOuter();
                }

                hzIntakeUltimateGoal.setIntakeReleaseOpen();
                hzAutonomousController.setMagazineToCollect();

                //Move to Launching Position
                parked = true;

                //Write last position to static class to be used as initial position in TeleOp
                HzGameFieldUltimateGoal.currentPose = driveTrain.getPoseEstimate();
                HzGameFieldUltimateGoal.poseSetInAutonomous = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        //Write last position to static class to be used as initial position in TeleOp
        HzGameFieldUltimateGoal.currentPose = driveTrain.getPoseEstimate();
        HzGameFieldUltimateGoal.poseSetInAutonomous = true;
    }

    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoInner(){

        //Initial Wait time logic
        if (!hzAutonomousController.launchHighGoalOrPowerShot){
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-53, af *10, Math.toRadians(af * 90))) //-40,6
                    .build();
            driveTrain.followTrajectory(traj);
            hzWait(19000); // starts at 7 sec balance, 4 seconds to execute, 3 seconds at the end
            //hzWait(1000);
        } else {
            if (!hzAutonomousController.dropFirstWobbleGoal){
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-53, af *10, Math.toRadians(af * 90))) //-40,6
                        .build();
                driveTrain.followTrajectory(traj);
                hzWait(7000); // 10 seconds in the end
            } else{
                if (!hzAutonomousController.pickRingFromTargetMarker){
                    //hzWait(1500);//STATE TESTING
                } else {
                    //ADD CUSTOM WAIT TIME HERE
                }
            }
        }
        // Move to launch position and launch rings to High Goal or Powershots
        if (!hzAutonomousController.launchHighGoalOrPowerShot) {
            runInnerOnlyLaunchPark(0);
            return;
        } else {
            // Move to launch position and launch rings to High Goal or Powershots
            if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
                // Set magazine to Launch in case it slipped
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();

                //Intermediary position to move away from alliance robot.
                //For option of second wobble goal drop, this is avoided to save time
                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    if (!hzAutonomousController.dropFirstWobbleGoal) {
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-54, af *10, Math.toRadians(af * 90))) //-40,6
                                .build();
                        driveTrain.followTrajectory(traj);
                    } else {
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                                .build();
                        driveTrain.followTrajectory(traj);
                    }
                }
                hzAutonomousController.setMagazineToLaunch();
                //Move to position to launch rings
                if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, 14, Math.toRadians(19)))//-10  ORIGINAL.. IST RING NOT WORKING
                            .lineToLinearHeading(new Pose2d(-6, 14, Math.toRadians(19)))//-10
                            .build();
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, -14, Math.toRadians(-12)))//-10 ORIGINAL.. IST RING NOT WORKING
                            .lineToLinearHeading(new Pose2d(-13, -12, Math.toRadians(-14))) //STATE TESTING
                            .build();
                }
                driveTrain.followTrajectory(traj);

                launch3RingsToHighGoal();
            } else {
                hzAutonomousController.setLaunchTargetPowerShot1();

                //Intermediary position to move away from alliance robot.
                //For option of second wobble goal drop, this is avoided to save time
                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                            .build();
                    driveTrain.followTrajectory(traj);
                }
                //Move to position to launch rings

                if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, af * 15, Math.toRadians(af * 5)))
                            .lineToLinearHeading(new Pose2d(-10, af * 19, Math.toRadians(af * 5)))
                            .build();
                    driveTrain.followTrajectory(traj);
                    //Set turn angles prior to launching for each of the power shorts
                    turnAnglePowershot12 = Math.toRadians(af * -5);
                    turnAnglePowershot23 = Math.toRadians(af * -7);
                } else {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, af * 15, Math.toRadians(af * 0)))
                            .build();
                    driveTrain.followTrajectory(traj);
                    //Set turn angles prior to launching for each of the power shorts
                    turnAnglePowershot12 = Math.toRadians(af * -5);
                    turnAnglePowershot23 = Math.toRadians(af * -7);
                }


                launch3RingsToPowerShots();
            }
        }

        // For case when wobble goal is not to be done and only park.

        if (!hzAutonomousController.dropFirstWobbleGoal) {
            runInnerOnlyLaunchPark(0);
            return;
        } else {

            // Move to drop wobble goal on target
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                switch (targetZone) {
                    case A:
                        if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    //.lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -45))) //y:51
                                    .lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -45))) //y:51
                                    .build();
                            driveTrain.followTrajectory(traj);
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    //.lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -45))) //y:51
                                    .lineToSplineHeading(new Pose2d(48, af * 53, Math.toRadians(af * -45))) //y:51
                                    .build();
                            driveTrain.followTrajectory(traj);

                        }
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                //.lineToSplineHeading(new Pose2d(27, af * 48, Math.toRadians(af * -45)))//43
                                .lineToSplineHeading(new Pose2d(27, af * 53, Math.toRadians(af * -45)))//43
                                .build();
                        driveTrain.followTrajectory(traj);
                        break;
                    case B:
                        if (!hzAutonomousController.pickRingFromTargetMarker){
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(35, af* 23, Math.toRadians(-90))) // STATE x:38 y:20
                                    .build();
                            driveTrain.followTrajectory(traj);
                        } else {
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(17, af * 30, Math.toRadians(af * -135)))
                                    .build();
                            driveTrain.followTrajectory(traj);
                        }
                        break;
                    case C:
                        if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -90))) //y:51
                                    .build();
                            driveTrain.followTrajectory(traj);
                        }
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(56, af * 45, Math.toRadians(af * -90))) //y:51 STATE y:43
                                .build();
                        driveTrain.followTrajectory(traj);
                        break;
                }
            } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                switch (targetZone) {
                    case A:
                        if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -45))) //y:51
                                    .build();
                            driveTrain.followTrajectory(traj);
                        }
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(33, af * 48, Math.toRadians(af * -45)))//43
                                .build();
                        driveTrain.followTrajectory(traj);
                        break;
                    case B:
                        if (!hzAutonomousController.pickRingFromTargetMarker){
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(36, af* 17, Math.toRadians(af* -90)))
                                    .build();
                            driveTrain.followTrajectory(traj);
                        } else {
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(21, af * 21, Math.toRadians(af * -135)))
                                    .build();
                            driveTrain.followTrajectory(traj);
                        }
                        break;
                    case C:
                        if ((!hzAutonomousController.pickAndDropSecondWobbleGoal) && (!hzAutonomousController.pickRingFromTargetMarker)) {
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -90))) //y:51
                                    .build();
                            driveTrain.followTrajectory(traj);
                        }
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(50, af * 45, Math.toRadians(af * -135))) //y:51
                                .build();
                        driveTrain.followTrajectory(traj);
                        break;
                }
            }

            dropWobbleGoalInTarget();

            if ((hzAutonomousController.pickRingFromTargetMarker == false) || (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A)) { // Park
                if (hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    runInnerPickAndDropSecondWobbleGoalAndPark();
                } else {
                    runInnerOnlyLaunchPark(3000);
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
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                //.lineToLinearHeading(new Pose2d(-8, af * 37, Math.toRadians(5))) //STATE y: 31
                                .lineToLinearHeading(new Pose2d(-8, af * 37, Math.toRadians(5))) //STATE y: 31
                                .build();
                        driveTrain.followTrajectory(traj);
                    } else {
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-8, af * 34, Math.toRadians(0))) //STATE x :-13angle:5
                                .build();
                        driveTrain.followTrajectory(traj);
                    }

                    launch3RingsToHighGoal();

                    if (hzAutonomousController.pickAndDropSecondWobbleGoal == true) {
                        runInnerPickAndDropSecondWobbleGoalAndPark();
                    } else {
                        //Park
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13, af * 36, Math.toRadians(af * 0)))
                                .build();
                        driveTrain.followTrajectory(traj);
                    }
                } else { //Move to baseline and park in safe zone
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-50, af * 16, Math.toRadians(af * -90)))
                            .build();
                    driveTrain.followTrajectory(traj);
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(10, af * 0, Math.toRadians(af * 0)))
                            .build();
                    driveTrain.followTrajectory(traj);
                    hzAutonomousController.setIntakeStop();
                }
            }
        }
    }

    public void runInnerOnlyLaunchPark(int waitInBetween){
        if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE &&
                targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A){
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(48, af * 30, Math.toRadians(af * -45)))
                    .build();
            driveTrain.followTrajectory(traj);
        } else {
            //Move towards base line away from other robot and Park
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(53, af * 15, Math.toRadians(af * -45))) //STATE x:50 y:15
                    .build();
            driveTrain.followTrajectory(traj);
        }
        hzWait(waitInBetween);
        if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 17, Math.toRadians(af * 0)))
                    .build();
            driveTrain.followTrajectory(traj);
        } else {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 13, Math.toRadians(af * 0)))
                    .build();
            driveTrain.followTrajectory(traj);
        }
    }

    /**
     * Path for picking and dropping second wobble goal for Inner Start position
     */
    public void runInnerPickAndDropSecondWobbleGoalAndPark() {
        hzAutonomousController.setMoveArmPickWobble();
        hzAutonomousController.runOpenGrip();
        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-33, af*50, Math.toRadians(af*0)))
                .build();
        driveTrain.followTrajectory(traj);
        hzWait(200);
        hzAutonomousController.runCloseGrip();
        hzWait(400);
        hzAutonomousController.setMoveArmHoldUpWobbleRing();
        hzWait(500);
        switch (targetZone) {
            case A:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*20, Math.toRadians(af*-60)))
                        .build();
                driveTrain.followTrajectory(traj);
                break;
            case B:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*33, Math.toRadians(af*180)))
                        .build();
                driveTrain.followTrajectory(traj);
                break;
            case C:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(42, af*50, Math.toRadians(af*-135)))
                        .build();
                driveTrain.followTrajectory(traj);
                break;
        }
        dropWobbleGoalInTarget();
        //Park
        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                    .build();
            driveTrain.followTrajectory(traj);
        }
    }

    /**
     * Path and actions for autonomous mode starting from Outer start position
     */
    public void runAutoOuter(){

        //Initial Wait time logic
        if (!hzAutonomousController.launchHighGoalOrPowerShot){
            hzWait(17000); // starts at 7 sec balance, 4 seconds to execute, 3 seconds at the end
        } else {
            if (!hzAutonomousController.dropFirstWobbleGoal){
                hzWait(7000); // 3 seconds in the end
            } else{
                if (!hzAutonomousController.pickRingFromTargetMarker){
                    hzWait(3000);
                } else {
                    //ADD CUSTOM WAIT TIME HERE
                }
            }
        }

        // Move to launch position and launch rings to High Goal or Powershots
        if (!hzAutonomousController.launchHighGoalOrPowerShot) {
            runOuterOnlyLaunchPark(0);
            return;
        } else {
            // Move to launch position and launch rings to High Goal or Powershots
            if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
                // Set magazine to Launch in case it slipped
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();
                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-40, af * 53, Math.toRadians(af * -45)))
                            //.lineToLinearHeading(new Pose2d(-40, af * 50, Math.toRadians(af * -45)))
                            .build();
                    driveTrain.followTrajectory(traj);
                }
                hzAutonomousController.setMagazineToLaunch();
                if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    //PATGAMES : MOVE ROBOT TO LAST LANE and not obstruct to middle lane
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, 52, Math.toRadians(-12)))
                            .lineToLinearHeading(new Pose2d(-6, 52, Math.toRadians(-12)))
                            .build();
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    //PATGAMES : MOVE ROBOT TO LAST LANE and not obstruct to middle lane
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, -52, Math.toRadians(17)))//-10
                            .lineToLinearHeading(new Pose2d(-6, -52, Math.toRadians(18)))
                            //.lineToLinearHeading(new Pose2d(-3, -52, Math.toRadians(19)))
                            .build();
                }
                driveTrain.followTrajectory(traj);

                launch3RingsToHighGoal();
            } else {
                hzAutonomousController.setLaunchTargetPowerShot1();
                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-40, af * 53, Math.toRadians(af * -45)))
                            .build();
                    driveTrain.followTrajectory(traj);
                }
                /*traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-10, af * 50, Math.toRadians(af * -22)), Math.toRadians(0))
                        .build();
                hzDrive.followTrajectory(traj);*/
                hzAutonomousController.setMagazineToLaunch();
                if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    //PATGAMES : MOVE ROBOT TO LAST LANE and not obstruct to middle lane
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, 52, Math.toRadians(-20)))
                            .build();
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    //PATGAMES : MOVE ROBOT TO LAST LANE and not obstruct to middle lane
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, -52, Math.toRadians(28)))//-10
                            .build();
                }
                driveTrain.followTrajectory(traj);

                //Set turn angles prior to launching
                turnAnglePowershot12 = Math.toRadians(af*-5);
                turnAnglePowershot23 = Math.toRadians(af*-7);
                launch3RingsToPowerShots();
            }
        }

        if (!hzAutonomousController.dropFirstWobbleGoal) {
            runOuterOnlyLaunchPark(5000);
            return;
        } else {
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                switch (targetZone) {
                    case A:
                        //patgames
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-3, 52, Math.toRadians(-150)))
                                .build();
                        driveTrain.followTrajectory(traj);
                        break;
                    case B:
                        if (!hzAutonomousController.pickRingFromTargetMarker){
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(38, 50, Math.toRadians(90)))
                                    .build();
                            driveTrain.followTrajectory(traj);
                        } else {
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(18, 33, Math.toRadians(180)))
                                    .build();
                            driveTrain.followTrajectory(traj);
                        }
                        break;
                    case C:
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(40, 52, Math.toRadians(-150)))
                                .build();
                        driveTrain.followTrajectory(traj);
                        break;
                }
            } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                switch (targetZone) {
                    case A:
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(0, -53, Math.toRadians(180)))
                                .build();
                        driveTrain.followTrajectory(traj);
                        break;
                    case B:
                        if (!hzAutonomousController.pickRingFromTargetMarker){
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(31, -50, Math.toRadians(-90))) //STATE x:32
                                    .build();
                            driveTrain.followTrajectory(traj);
                        } else {
                            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                    //.lineToSplineHeading(new Pose2d(18, -30, Math.toRadians(180)))
                                    .lineToSplineHeading(new Pose2d(18, -36, Math.toRadians(180)))
                                    .build();
                            driveTrain.followTrajectory(traj);
                        }
                        break;
                    case C:
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(44, -53, Math.toRadians(180))) //STATE x:47
                                .build();
                        driveTrain.followTrajectory(traj);
                        break;
                }
            }

            dropWobbleGoalInTarget();
            if ((hzAutonomousController.pickRingFromTargetMarker == false) || (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A)) { // Park
                if (hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    runOuterPickAndDropSecondWobbleGoalAndPark();
                } else {
                    runOuterOnlyLaunchPark(8000);
                }
                return;
            } else { //Target Zone is B or C and pickRingFromTargetMarker == True
                //Pick rings from Target Marker
                runPickRingsFromTargetMarker();

                if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal) {
                    hzAutonomousController.setIntakeStop();
                    hzAutonomousController.setMagazineToLaunch();
                    hzAutonomousController.setLaunchTargetHighGoal();

                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-8, af * 31, Math.toRadians(2))) //STATE x: -13
                            .build();
                    driveTrain.followTrajectory(traj);

                    launch3RingsToHighGoal();

                    if (hzAutonomousController.pickAndDropSecondWobbleGoal == true) {
                        runOuterPickAndDropSecondWobbleGoalAndPark();
                    } else {

                        //Park
                        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13, af * 54, Math.toRadians(af * 0)))
                                //.lineToSplineHeading(new Pose2d(13, af * 46, Math.toRadians(af * 0)))
                                .build();
                        driveTrain.followTrajectory(traj);
                    }
                } else { //Move to baseline and park in safe zone
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-50, af * 16, Math.toRadians(af * -90)))
                            .build();
                    driveTrain.followTrajectory(traj);
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(13, af * 10, Math.toRadians(af * 0)))
                            .build();
                    driveTrain.followTrajectory(traj);
                    hzAutonomousController.setIntakeStop();
                }
            }
        }
    }

    public void runPickRingsFromTargetMarker(){
        hzIntakeUltimateGoal.setIntakeReleaseOpen();
        hzAutonomousController.setIntakeStart();
        hzWait(500);

        //Move to Position to pick rings
        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.B) {

            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 33.5, Math.toRadians(af * 180)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * 180)))
                        .lineToLinearHeading(new Pose2d(-22, af * 33.5, Math.toRadians(af * 180)))
                        .build();
                driveTrain.followTrajectory(traj);
            } else {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 35.5, Math.toRadians(af * 180)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * 180)))
                        .lineToLinearHeading(new Pose2d(-22, af * 35.5, Math.toRadians(af * 180)))
                        .build();
                driveTrain.followTrajectory(traj);
            }
        }

        // Spline to (24,24,0)
        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {


            //TEST
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 33.5, Math.toRadians(af * -180)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-11, af * 33.5, Math.toRadians(af * -180)))
                        .build();
                driveTrain.followTrajectory(traj);

                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * -180)))
                            .lineToLinearHeading(new Pose2d(-22, af * 35.5, Math.toRadians(af * -180)))
                            .build();
                    driveTrain.followTrajectory(traj);
                }
            } else {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 35.5, Math.toRadians(af * -180)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-11, af * 35.5, Math.toRadians(af * -180)))
                        .build();
                driveTrain.followTrajectory(traj);

                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * -180)))
                            .lineToLinearHeading(new Pose2d(-22, af * 35.5, Math.toRadians(af * -180)))
                            .build();
                    driveTrain.followTrajectory(traj);
                }
            }


            /*traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    //.lineToLinearHeading(new Pose2d(-33, af * 34.5, Math.toRadians(af * -180))) //x33
                    .lineToLinearHeading(new Pose2d(-30, af * 34.5, Math.toRadians(af * -180))) //x33
                    .build();
            hzDrive.followTrajectory(traj);*/
        }
        hzWait(300);
    }

    public void runOuterOnlyLaunchPark(int waitInBetween){
        //Park in or Next to Target A

        //Intermediary stop position in the back of the firld
        //PATGAMES :
        if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-45, af * 49, Math.toRadians(af * -25)))
                    .build();
            driveTrain.followTrajectory(traj);
        } else {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-45, af * 54, Math.toRadians(af * -25)))
                    .build();
            driveTrain.followTrajectory(traj);
        }
        hzWait(waitInBetween);

        /*Comment this section if alternate is used*/
        if (targetZone != HzGameFieldUltimateGoal.TARGET_ZONE.A) {
            //Park in Target Zone A
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 52, Math.toRadians(af * 0))) // In Target A
                    .build();
            driveTrain.followTrajectory(traj);
        } else {
            //Park Next to Target Zone A
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(16, af * 35, Math.toRadians(af * 0))) // Near Target A //STATE x:13
                    .build();
            driveTrain.followTrajectory(traj);
        }
        //*/

        /*Alternate Park Near center : Use if Target is not visible. or want to park in middle
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-45, af * 12, Math.toRadians(af * 0))) //  Towards Middle
                .build();
        hzDrive.followTrajectory(traj);
        hzWait(0);
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(13, af * 12, Math.toRadians(af * 0))) //  Towards Middle
                .build();
        hzDrive.followTrajectory(traj);
        //*/

    }

    /**
     * Path for picking and dropping second wobble goal for Outer Start position
     */
    public void runOuterPickAndDropSecondWobbleGoalAndPark() {
        hzAutonomousController.setMoveArmPickWobble();
        hzAutonomousController.runOpenGrip();
        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-31, af*23, Math.toRadians(af*0)))
                .build();
        driveTrain.followTrajectory(traj);
        hzWait(200);
        hzAutonomousController.runCloseGrip();
        hzWait(400);
        hzAutonomousController.setMoveArmHoldUpWobbleRing();
        hzWait(500);
        switch (targetZone) {
            case A:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*20, Math.toRadians(af*-60)))
                        .build();
                driveTrain.followTrajectory(traj);
                break;
            case B:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*33, Math.toRadians(af*180)))
                        .build();
                driveTrain.followTrajectory(traj);
                break;
            case C:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(37, af*50, Math.toRadians(af*-135)))
                        .build();
                driveTrain.followTrajectory(traj);
                break;
        }
        dropWobbleGoalInTarget();
        //Park
        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                    .build();
            driveTrain.followTrajectory(traj);
        }
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

        driveTrain.turn(turnAnglePowershot12);
        hzAutonomousController.setLaunchTargetPowerShot1();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();

        driveTrain.turn(turnAnglePowershot23);
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

    /**
     * Method that take key pad inputs to select the Autonomous options
     */
    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "4:47 :: 2/13");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Blue: (X),    Red: (B))");
        telemetry.update();


        while (!isStopRequested()) {
            if (hzGamepadControllerUltimateGoal.getButtonBPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE;
                HzGameFieldUltimateGoal.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                HzGameFieldUltimateGoal.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }

        telemetry.update();
        hzWait(200);

        //***** Select Start Pose ******
        telemetry.addData("Enter Start Pose :", "(Inner: (A) ,    Outer: (Y))");
        while (!isStopRequested()) {
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    HzGameFieldUltimateGoal.startPosition = HzGameFieldUltimateGoal.START_POSITION.INNER;
                    startPose = HzGameFieldUltimateGoal.RED_INNER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    HzGameFieldUltimateGoal.startPosition = HzGameFieldUltimateGoal.START_POSITION.OUTER;
                    startPose = HzGameFieldUltimateGoal.RED_OUTER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    HzGameFieldUltimateGoal.startPosition = HzGameFieldUltimateGoal.START_POSITION.INNER;
                    startPose = HzGameFieldUltimateGoal.BLUE_INNER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    HzGameFieldUltimateGoal.startPosition = HzGameFieldUltimateGoal.START_POSITION.OUTER;
                    startPose = HzGameFieldUltimateGoal.BLUE_OUTER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            telemetry.update();
        }
        telemetry.update();


        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameFieldUltimateGoal.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ",activeWebcam);
            telemetry.addLine();
            telemetry.addData("Please select launch target : ", "");
            telemetry.addData("     (X) for Powershot","");
            telemetry.addData("     (B) for High Goal","");
            telemetry.addData("     (A) for No launch - Only park","");
            if (hzGamepadControllerUltimateGoal.getButtonBPress()) {
                hzAutonomousController.launchHighGoalOrPowerShot = true;
                hzAutonomousController.autoLaunchAim = HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutonomousController.autoLaunchAim);
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                hzAutonomousController.launchHighGoalOrPowerShot = true;
                hzAutonomousController.autoLaunchAim = HzAutonomousControllerUltimateGoal.AutoLaunchAim.POWERSHOT;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutonomousController.autoLaunchAim);
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                /*hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);*/
                telemetry.addData("No Launch, Only Park : ","");
                hzAutonomousController.launchHighGoalOrPowerShot = false;
                hzAutonomousController.dropFirstWobbleGoal = false;
                hzAutonomousController.pickRingFromTargetMarker = false;
                hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = false;
                hzAutonomousController.pickAndDropSecondWobbleGoal = false;
                break;
            }
            telemetry.update();
            //hzWait(200);
        }

        if (hzAutonomousController.launchHighGoalOrPowerShot) {
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance :", HzGameFieldUltimateGoal.playingAlliance);
                telemetry.addData("startPose : ", startPose);
                telemetry.addData("activeWebcam : ", activeWebcam);
                telemetry.addLine();

                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutonomousController.autoLaunchAim);
                telemetry.addData("Please select option for Picking rings from target markers and launching : ", "");
                telemetry.addData("    (Y):", "Launch and park");
                telemetry.addData("    (A):", "Launch, drop WG and park");
                telemetry.addData("    (X):", "Launch, drop WG, Pick rings, launch and park");
                telemetry.addData("    (B):", "Launch, drop WG, Pick rings, launch, move WG2 and park");

                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    hzAutonomousController.dropFirstWobbleGoal = false;
                    hzAutonomousController.pickRingFromTargetMarker = false;
                    hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = false;
                    hzAutonomousController.pickAndDropSecondWobbleGoal = false;
                    telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutonomousController.pickRingFromTargetMarker);
                    telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal);
                    telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutonomousController.pickAndDropSecondWobbleGoal);
                    break;
                }

                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    hzAutonomousController.dropFirstWobbleGoal = true;
                    hzAutonomousController.pickRingFromTargetMarker = false;
                    hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = false;
                    hzAutonomousController.pickAndDropSecondWobbleGoal = false;
                    telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutonomousController.dropFirstWobbleGoal);
                    telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutonomousController.pickRingFromTargetMarker);
                    telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal);
                    telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutonomousController.pickAndDropSecondWobbleGoal);
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                    hzAutonomousController.dropFirstWobbleGoal = true;
                    hzAutonomousController.pickRingFromTargetMarker = true;
                    hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = true;
                    hzAutonomousController.pickAndDropSecondWobbleGoal = false;
                    telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutonomousController.dropFirstWobbleGoal);
                    telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutonomousController.pickRingFromTargetMarker);
                    telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal);
                    telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutonomousController.pickAndDropSecondWobbleGoal);
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonBPress()) {
                    hzAutonomousController.dropFirstWobbleGoal = true;
                    hzAutonomousController.pickRingFromTargetMarker = true;
                    hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = true;
                    hzAutonomousController.pickAndDropSecondWobbleGoal = true;
                    telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutonomousController.dropFirstWobbleGoal);
                    telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutonomousController.pickRingFromTargetMarker);
                    telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal);
                    telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutonomousController.pickAndDropSecondWobbleGoal);
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
        }
        telemetry.update();
        //sleep(200);
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
        telemetry.addData("Drive Mode : ", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

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
        telemetry.addData("hzDrive.drivePointToAlign : ", driveTrain.drivePointToAlign);

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

