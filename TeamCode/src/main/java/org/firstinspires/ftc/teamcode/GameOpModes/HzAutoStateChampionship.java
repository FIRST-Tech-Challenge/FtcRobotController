package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.HzArm;
import org.firstinspires.ftc.teamcode.SubSystems.HzAutoControl;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.SubSystems.HzGameField;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad;
import org.firstinspires.ftc.teamcode.SubSystems.HzIntake;
import org.firstinspires.ftc.teamcode.SubSystems.HzLaunchController;
import org.firstinspires.ftc.teamcode.SubSystems.HzLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.HzMagazine;
import org.firstinspires.ftc.teamcode.SubSystems.HzVuforia;

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
@Autonomous(name = "Hazmat State Championship", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp RR")
public class HzAutoStateChampionship extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepad hzGamepad;
    public HzAutoControl hzAutoControl;
    public HzDrive hzDrive;
    public HzMagazine hzMagazine;
    public HzIntake hzIntake;
    public HzLaunchController hzLaunchController;
    public HzLauncher hzLauncher;
    public HzArm hzArm;

    public HzVuforia hzVuforia;
    public Pose2d startPose = HzGameField.BLUE_INNER_START_LINE;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public HzGameField.TARGET_ZONE targetZone = HzGameField.TARGET_ZONE.A;
    public HzVuforia.ACTIVE_WEBCAM activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;

    double af = HzGameField.ALLIANCE_FACTOR;

    double turnAnglePowershot12 = Math.toRadians(-5);
    double turnAnglePowershot23 = Math.toRadians(-5);
    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize HW
        hzDrive = new HzDrive(hardwareMap);
        hzMagazine = new HzMagazine(hardwareMap);
        hzIntake = new HzIntake(hardwareMap);

        hzLauncher = new HzLauncher(hardwareMap);
        hzArm = new HzArm(hardwareMap);
        hzLaunchController = new HzLaunchController(hardwareMap, hzLauncher, hzIntake, hzMagazine, hzDrive);
        hzGamepad = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);
        hzAutoControl = new HzAutoControl(hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        //Key Pay inputs to select Game Plan;
        selectGamePlan();
        hzVuforia = new HzVuforia(hardwareMap, activeWebcam);
        af = HzGameField.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        hzVuforia.activateVuforiaTensorFlow();

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        hzIntake.setIntakeReleaseHold();
        hzAutoControl.setMagazineToLaunch();

        hzLauncher.flyWheelVelocityHighGoal = 1430;
        hzLauncher.flyWheelVelocityPowerShot = 1330;

        while (hzMagazine.magazineLaunchTouchSensor.isPressed() == false) {
            hzAutoControl.setMagazineToLaunch();
            if (isStopRequested()) return;
        }

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow
            targetZone = hzVuforia.runVuforiaTensorFlow();

            if (!parked){
                hzAutoControl.setMagazineToLaunch();
                hzAutoControl.runAutoControl();
            }

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
            hzLaunchController.launchMode = HzLaunchController.LAUNCH_MODE.MANUAL;

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {

                hzVuforia.deactivateVuforiaTensorFlow();

                if (HzGameField.startPosition == HzGameField.START_POSITION.INNER) {
                    runAutoInner();
                } else { //HzGameField.startPosition == HzGameField.START_POSITION.OUTER
                    runAutoOuter();
                }

                hzIntake.setIntakeReleaseOpen();
                hzAutoControl.setMagazineToCollect();

                //Move to Launching Position
                parked = true;

                //Write last position to static class to be used as initial position in TeleOp
                HzGameField.currentPose = hzDrive.getPoseEstimate();
                HzGameField.poseSetInAutonomous = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        //Write last position to static class to be used as initial position in TeleOp
        HzGameField.currentPose = hzDrive.getPoseEstimate();
        HzGameField.poseSetInAutonomous = true;
    }

    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoInner(){

        //Initial Wait time logic
        if (!hzAutoControl.launchHighGoalOrPowerShot){
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-53, af *10, Math.toRadians(af * 90))) //-40,6
                    .build();
            hzDrive.followTrajectory(traj);
            hzWait(19000); // starts at 7 sec balance, 4 seconds to execute, 3 seconds at the end
            //hzWait(1000);
        } else {
            if (!hzAutoControl.dropFirstWobbleGoal){
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-53, af *10, Math.toRadians(af * 90))) //-40,6
                        .build();
                hzDrive.followTrajectory(traj);
                hzWait(7000); // 10 seconds in the end
            } else{
                if (!hzAutoControl.pickRingFromTargetMarker){
                    //hzWait(1500);//STATE TESTING
                } else {
                    //ADD CUSTOM WAIT TIME HERE
                }
            }
        }
        // Move to launch position and launch rings to High Goal or Powershots
        if (!hzAutoControl.launchHighGoalOrPowerShot) {
            runInnerOnlyLaunchPark(0);
            return;
        } else {
            // Move to launch position and launch rings to High Goal or Powershots
            if (hzAutoControl.autoLaunchAim == HzAutoControl.AutoLaunchAim.HIGHGOAL) {
                // Set magazine to Launch in case it slipped
                hzAutoControl.setMagazineToLaunch();
                hzAutoControl.setLaunchTargetHighGoal();

                //Intermediary position to move away from alliance robot.
                //For option of second wobble goal drop, this is avoided to save time
                if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                    if (!hzAutoControl.dropFirstWobbleGoal) {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-54, af *10, Math.toRadians(af * 90))) //-40,6
                                .build();
                        hzDrive.followTrajectory(traj);
                    } else {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                                .build();
                        hzDrive.followTrajectory(traj);
                    }
                }
                hzAutoControl.setMagazineToLaunch();
                //Move to position to launch rings
                if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, 14, Math.toRadians(19)))//-10  ORIGINAL.. IST RING NOT WORKING
                            .lineToLinearHeading(new Pose2d(-6, 14, Math.toRadians(19)))//-10
                            .build();
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, -14, Math.toRadians(-12)))//-10 ORIGINAL.. IST RING NOT WORKING
                            .lineToLinearHeading(new Pose2d(-13, -12, Math.toRadians(-14))) //STATE TESTING
                            .build();
                }
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();
            } else {
                hzAutoControl.setLaunchTargetPowerShot1();

                //Intermediary position to move away from alliance robot.
                //For option of second wobble goal drop, this is avoided to save time
                if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                            .build();
                    hzDrive.followTrajectory(traj);
                }
                //Move to position to launch rings

                if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
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

        if (!hzAutoControl.dropFirstWobbleGoal) {
            runInnerOnlyLaunchPark(0);
            return;
        } else {

            // Move to drop wobble goal on target
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                switch (targetZone) {
                    case A:
                        if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    //.lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -45))) //y:51
                                    .lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -45))) //y:51
                                    .build();
                            hzDrive.followTrajectory(traj);
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    //.lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -45))) //y:51
                                    .lineToSplineHeading(new Pose2d(48, af * 53, Math.toRadians(af * -45))) //y:51
                                    .build();
                            hzDrive.followTrajectory(traj);

                        }
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                //.lineToSplineHeading(new Pose2d(27, af * 48, Math.toRadians(af * -45)))//43
                                .lineToSplineHeading(new Pose2d(27, af * 53, Math.toRadians(af * -45)))//43
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        if (!hzAutoControl.pickRingFromTargetMarker){
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(35, af* 23, Math.toRadians(-90))) // STATE x:38 y:20
                                    .build();
                            hzDrive.followTrajectory(traj);
                        } else {
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(20, af * 30, Math.toRadians(af * -135)))
                                    .build();
                            hzDrive.followTrajectory(traj);
                        }
                        break;
                    case C:
                        if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -90))) //y:51
                                    .build();
                            hzDrive.followTrajectory(traj);
                        }
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(56, af * 45, Math.toRadians(af * -90))) //y:51 STATE y:43
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                }
            } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                switch (targetZone) {
                    case A:
                        if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -45))) //y:51
                                    .build();
                            hzDrive.followTrajectory(traj);
                        }
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(33, af * 48, Math.toRadians(af * -45)))//43
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        if (!hzAutoControl.pickRingFromTargetMarker){
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(36, af* 17, Math.toRadians(af* -90)))
                                    .build();
                            hzDrive.followTrajectory(traj);
                        } else {
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(21, af * 21, Math.toRadians(af * -135)))
                                    .build();
                            hzDrive.followTrajectory(traj);
                        }
                        break;
                    case C:
                        if ((!hzAutoControl.pickAndDropSecondWobbleGoal) && (!hzAutoControl.pickRingFromTargetMarker)) {
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(48, af * 15, Math.toRadians(af * -90))) //y:51
                                    .build();
                            hzDrive.followTrajectory(traj);
                        }
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(50, af * 45, Math.toRadians(af * -135))) //y:51
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                }
            }

            dropWobbleGoalInTarget();

            if ((hzAutoControl.pickRingFromTargetMarker == false) || (targetZone == HzGameField.TARGET_ZONE.A)) { // Park
                if (hzAutoControl.pickAndDropSecondWobbleGoal) {
                    runInnerPickAndDropSecondWobbleGoalAndPark();
                } else {
                    runInnerOnlyLaunchPark(3000);
                }
                return;
            } else { //Target Zone is B or C and pickRingFromTargetMarker == True
                //Pick rings from Target Marker
                runPickRingsFromTargetMarker();

                if (hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal) {
                    hzAutoControl.setIntakeStop();
                    hzAutoControl.setMagazineToLaunch();
                    hzAutoControl.setLaunchTargetHighGoal();

                    if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                //.lineToLinearHeading(new Pose2d(-8, af * 37, Math.toRadians(5))) //STATE y: 31
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

                    if (hzAutoControl.pickAndDropSecondWobbleGoal == true) {
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
                    hzAutoControl.setIntakeStop();
                }
            }
        }
    }

    public void runInnerOnlyLaunchPark(int waitInBetween){
        if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE &&
                targetZone == HzGameField.TARGET_ZONE.A){
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(48, af * 30, Math.toRadians(af * -45)))
                    .build();
            hzDrive.followTrajectory(traj);
        } else {
            //Move towards base line away from other robot and Park
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(53, af * 15, Math.toRadians(af * -45))) //STATE x:50 y:15
                    .build();
            hzDrive.followTrajectory(traj);
        }
        hzWait(waitInBetween);
        if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 17, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
        } else {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 13, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
        }
    }

    /**
     * Path for picking and dropping second wobble goal for Inner Start position
     */
    public void runInnerPickAndDropSecondWobbleGoalAndPark() {
        hzAutoControl.setMoveArmPickWobble();
        hzAutoControl.runOpenGrip();
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-33, af*50, Math.toRadians(af*0)))
                .build();
        hzDrive.followTrajectory(traj);
        hzWait(200);
        hzAutoControl.runCloseGrip();
        hzWait(400);
        hzAutoControl.setMoveArmHoldUpWobbleRing();
        hzWait(500);
        switch (targetZone) {
            case A:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*20, Math.toRadians(af*-60)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*33, Math.toRadians(af*180)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(42, af*50, Math.toRadians(af*-135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
        }
        dropWobbleGoalInTarget();
        //Park
        if (targetZone == HzGameField.TARGET_ZONE.C) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
        }
    }

    /**
     * Path and actions for autonomous mode starting from Outer start position
     */
    public void runAutoOuter(){

        //Initial Wait time logic
        if (!hzAutoControl.launchHighGoalOrPowerShot){
            hzWait(17000); // starts at 7 sec balance, 4 seconds to execute, 3 seconds at the end
        } else {
            if (!hzAutoControl.dropFirstWobbleGoal){
                hzWait(7000); // 3 seconds in the end
            } else{
                if (!hzAutoControl.pickRingFromTargetMarker){
                    hzWait(3000);
                } else {
                    //ADD CUSTOM WAIT TIME HERE
                }
            }
        }

        // Move to launch position and launch rings to High Goal or Powershots
        if (!hzAutoControl.launchHighGoalOrPowerShot) {
            runOuterOnlyLaunchPark(0);
            return;
        } else {
            // Move to launch position and launch rings to High Goal or Powershots
            if (hzAutoControl.autoLaunchAim == HzAutoControl.AutoLaunchAim.HIGHGOAL) {
                // Set magazine to Launch in case it slipped
                hzAutoControl.setMagazineToLaunch();
                hzAutoControl.setLaunchTargetHighGoal();
                if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-40, af * 53, Math.toRadians(af * -45)))
                            //.lineToLinearHeading(new Pose2d(-40, af * 50, Math.toRadians(af * -45)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }
                hzAutoControl.setMagazineToLaunch();
                if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    //PATGAMES : MOVE ROBOT TO LAST LANE and not obstruct to middle lane
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, 52, Math.toRadians(-12)))
                            .lineToLinearHeading(new Pose2d(-6, 52, Math.toRadians(-12)))
                            .build();
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    //PATGAMES : MOVE ROBOT TO LAST LANE and not obstruct to middle lane
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-10, -52, Math.toRadians(17)))//-10
                            .lineToLinearHeading(new Pose2d(-6, -52, Math.toRadians(18)))
                            //.lineToLinearHeading(new Pose2d(-3, -52, Math.toRadians(19)))
                            .build();
                }
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();
            } else {
                hzAutoControl.setLaunchTargetPowerShot1();
                if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-40, af * 53, Math.toRadians(af * -45)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }
                /*traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-10, af * 50, Math.toRadians(af * -22)), Math.toRadians(0))
                        .build();
                hzDrive.followTrajectory(traj);*/
                hzAutoControl.setMagazineToLaunch();
                if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    //PATGAMES : MOVE ROBOT TO LAST LANE and not obstruct to middle lane
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, 52, Math.toRadians(-20)))
                            .build();
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    //PATGAMES : MOVE ROBOT TO LAST LANE and not obstruct to middle lane
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, -52, Math.toRadians(28)))//-10
                            .build();
                }
                hzDrive.followTrajectory(traj);

                //Set turn angles prior to launching
                turnAnglePowershot12 = Math.toRadians(af*-5);
                turnAnglePowershot23 = Math.toRadians(af*-7);
                launch3RingsToPowerShots();
            }
        }

        if (!hzAutoControl.dropFirstWobbleGoal) {
            runOuterOnlyLaunchPark(5000);
            return;
        } else {
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                switch (targetZone) {
                    case A:
                        //patgames
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-5, 52, Math.toRadians(-150)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        if (!hzAutoControl.pickRingFromTargetMarker){
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(38, 50, Math.toRadians(90)))
                                    .build();
                            hzDrive.followTrajectory(traj);
                        } else {
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(18, 36, Math.toRadians(180)))
                                    .build();
                            hzDrive.followTrajectory(traj);
                        }
                        break;
                    case C:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(40, 52, Math.toRadians(-150)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                }
            } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                switch (targetZone) {
                    case A:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-3, -53, Math.toRadians(180)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        if (!hzAutoControl.pickRingFromTargetMarker){
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(29, -50, Math.toRadians(-90))) //STATE x:32
                                    .build();
                            hzDrive.followTrajectory(traj);
                        } else {
                            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                    //.lineToSplineHeading(new Pose2d(18, -30, Math.toRadians(180)))
                                    .lineToSplineHeading(new Pose2d(14, -36, Math.toRadians(180)))
                                    .build();
                            hzDrive.followTrajectory(traj);
                        }
                        break;
                    case C:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(44, -53, Math.toRadians(180))) //STATE x:47
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                }
            }

            dropWobbleGoalInTarget();
            if ((hzAutoControl.pickRingFromTargetMarker == false) || (targetZone == HzGameField.TARGET_ZONE.A)) { // Park
                if (hzAutoControl.pickAndDropSecondWobbleGoal) {
                    runOuterPickAndDropSecondWobbleGoalAndPark();
                } else {
                    runOuterOnlyLaunchPark(8000);
                }
                return;
            } else { //Target Zone is B or C and pickRingFromTargetMarker == True
                //Pick rings from Target Marker
                runPickRingsFromTargetMarker();

                if (hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal) {
                    hzAutoControl.setIntakeStop();
                    hzAutoControl.setMagazineToLaunch();
                    hzAutoControl.setLaunchTargetHighGoal();

                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-8, af * 31, Math.toRadians(2))) //STATE x: -13
                            .build();
                    hzDrive.followTrajectory(traj);

                    launch3RingsToHighGoal();

                    if (hzAutoControl.pickAndDropSecondWobbleGoal == true) {
                        runOuterPickAndDropSecondWobbleGoalAndPark();
                    } else {

                        //Park
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13, af * 54, Math.toRadians(af * 0)))
                                //.lineToSplineHeading(new Pose2d(13, af * 46, Math.toRadians(af * 0)))
                                .build();
                        hzDrive.followTrajectory(traj);
                    }
                } else { //Move to baseline and park in safe zone
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-50, af * 16, Math.toRadians(af * -90)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(13, af * 10, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    hzAutoControl.setIntakeStop();
                }
            }
        }
    }

    public void runPickRingsFromTargetMarker(){
        hzIntake.setIntakeReleaseOpen();
        hzAutoControl.setIntakeStart();
        hzWait(500);

        //Move to Position to pick rings
        if (targetZone == HzGameField.TARGET_ZONE.B) {

            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 33.5, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * 180)))
                        .lineToLinearHeading(new Pose2d(-22, af * 33.5, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
            } else {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 35.5, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * 180)))
                        .lineToLinearHeading(new Pose2d(-22, af * 35.5, Math.toRadians(af * 180)))
                        .build();
                hzDrive.followTrajectory(traj);
            }
        }

        // Spline to (24,24,0)
        if (targetZone == HzGameField.TARGET_ZONE.C) {


            //TEST
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, af * 33.5, Math.toRadians(af * -180)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-11, af * 33.5, Math.toRadians(af * -180)))
                        .build();
                hzDrive.followTrajectory(traj);

                if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * -180)))
                            .lineToLinearHeading(new Pose2d(-22, af * 33.5, Math.toRadians(af * -180)))
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

                if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            //.lineToLinearHeading(new Pose2d(-22, af * 34.5, Math.toRadians(af * -180)))
                            .lineToLinearHeading(new Pose2d(-26, af * 35.5, Math.toRadians(af * -180)))
                            .build();
                    hzDrive.followTrajectory(traj);
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
        if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-45, af * 49, Math.toRadians(af * -25)))
                    .build();
            hzDrive.followTrajectory(traj);
        } else {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-45, af * 54, Math.toRadians(af * -25)))
                    .build();
            hzDrive.followTrajectory(traj);
        }
        hzWait(waitInBetween);

        /*Comment this section if alternate is used*/
        if (targetZone != HzGameField.TARGET_ZONE.A) {
            //Park in Target Zone A
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 52, Math.toRadians(af * 0))) // In Target A
                    .build();
            hzDrive.followTrajectory(traj);
        } else {
            //Park Next to Target Zone A
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(16, af * 35, Math.toRadians(af * 0))) // Near Target A //STATE x:13
                    .build();
            hzDrive.followTrajectory(traj);
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
        hzAutoControl.setMoveArmPickWobble();
        hzAutoControl.runOpenGrip();
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-31, af*23, Math.toRadians(af*0)))
                .build();
        hzDrive.followTrajectory(traj);
        hzWait(200);
        hzAutoControl.runCloseGrip();
        hzWait(400);
        hzAutoControl.setMoveArmHoldUpWobbleRing();
        hzWait(500);
        switch (targetZone) {
            case A:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*20, Math.toRadians(af*-60)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*33, Math.toRadians(af*180)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(37, af*50, Math.toRadians(af*-135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
        }
        dropWobbleGoalInTarget();
        //Park
        if (targetZone == HzGameField.TARGET_ZONE.C) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
        }
    }

    /**
     * Sequence of launching 3 rings to High Goal with time in between
     */
    public void launch3RingsToHighGoal1(){
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setMagazineToLaunch();
        hzWait(200);
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setMagazineToLaunch();
        hzWait(450); //400
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setMagazineToLaunch();
        hzWait(450);
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setMagazineToLaunch();
        hzWait(450);
        hzAutoControl.setRunLauncherTrue();

        // Run 4th just in case
        if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
            hzAutoControl.setMagazineToLaunch();
            hzWait(450);
            hzAutoControl.setRunLauncherTrue();
        }
        hzWait(600);

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();
    }

    /**
     * Sequence of launching 3 rings to High Goal
     */
    public void launch3RingsToHighGoal(){
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setMagazineToLaunch();
        //hzWait(1000); //400
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setRunLauncherTrue();
        //hzWait(200);
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setRunLauncherTrue();

        // Run 4th just in case
        /*if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
            hzAutoControl.setLaunchTargetHighGoal();
            hzAutoControl.setRunLauncherTrue();
            hzAutoControl.setLaunchTargetHighGoal();
            hzAutoControl.setRunLauncherTrue();
        }*/
        hzWait(400);

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();
    }

    /**
     * Sequence of launching 3 rings to PowerShots
     */
    public void launch3RingsToPowerShots(){
        hzAutoControl.setLaunchTargetPowerShot1();
        hzAutoControl.setMagazineToLaunch();
        hzWait(600);
        hzAutoControl.setRunLauncherTrue();

        hzDrive.turn(turnAnglePowershot12);
        hzAutoControl.setLaunchTargetPowerShot1();
        hzAutoControl.setMagazineToLaunch();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();

        hzDrive.turn(turnAnglePowershot23);
        hzAutoControl.setMagazineToLaunch();
        hzAutoControl.setLaunchTargetPowerShot1();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();
        hzWait(400);

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();

    }

    /**
     * Sequence of dropping Wobble Goal once it reaches target position
     */
    public void dropWobbleGoalInTarget(){
        hzAutoControl.setMoveArmDropWobbleAutonoumous();
        //hzWait(1000);
        hzWait(800);
        hzAutoControl.runOpenGrip();
        hzWait(700);
        //hzAutoControl.setMoveArmHoldUpWobbleRing();
        hzAutoControl.setMoveArmParked();
    }


    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void hzWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            hzAutoControl.runAutoControl();
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
            if (hzGamepad.getButtonBPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                HzGameField.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                HzGameField.ALLIANCE_FACTOR = 1;
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
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.INNER;
                    startPose = HzGameField.RED_INNER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonYPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.OUTER;
                    startPose = HzGameField.RED_OUTER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.INNER;
                    startPose = HzGameField.BLUE_INNER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonYPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.OUTER;
                    startPose = HzGameField.BLUE_OUTER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            telemetry.update();
        }
        telemetry.update();


        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameField.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ",activeWebcam);
            telemetry.addLine();
            telemetry.addData("Please select launch target : ", "");
            telemetry.addData("     (X) for Powershot","");
            telemetry.addData("     (B) for High Goal","");
            telemetry.addData("     (A) for No launch - Only park","");
            if (hzGamepad.getButtonBPress()) {
                hzAutoControl.launchHighGoalOrPowerShot = true;
                hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                hzAutoControl.launchHighGoalOrPowerShot = true;
                hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.POWERSHOT;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                break;
            }
            if (hzGamepad.getButtonAPress()) {
                /*hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);*/
                telemetry.addData("No Launch, Only Park : ","");
                hzAutoControl.launchHighGoalOrPowerShot = false;
                hzAutoControl.dropFirstWobbleGoal = false;
                hzAutoControl.pickRingFromTargetMarker = false;
                hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = false;
                hzAutoControl.pickAndDropSecondWobbleGoal = false;
                break;
            }
            telemetry.update();
            //hzWait(200);
        }

        if (hzAutoControl.launchHighGoalOrPowerShot) {
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance :", HzGameField.playingAlliance);
                telemetry.addData("startPose : ", startPose);
                telemetry.addData("activeWebcam : ", activeWebcam);
                telemetry.addLine();

                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                telemetry.addData("Please select option for Picking rings from target markers and launching : ", "");
                telemetry.addData("    (Y):", "Launch and park");
                telemetry.addData("    (A):", "Launch, drop WG and park");
                telemetry.addData("    (X):", "Launch, drop WG, Pick rings, launch and park");
                telemetry.addData("    (B):", "Launch, drop WG, Pick rings, launch, move WG2 and park");

                if (hzGamepad.getButtonYPress()) {
                    hzAutoControl.dropFirstWobbleGoal = false;
                    hzAutoControl.pickRingFromTargetMarker = false;
                    hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = false;
                    hzAutoControl.pickAndDropSecondWobbleGoal = false;
                    telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                    telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                    telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutoControl.pickAndDropSecondWobbleGoal);
                    break;
                }

                if (hzGamepad.getButtonAPress()) {
                    hzAutoControl.dropFirstWobbleGoal = true;
                    hzAutoControl.pickRingFromTargetMarker = false;
                    hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = false;
                    hzAutoControl.pickAndDropSecondWobbleGoal = false;
                    telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutoControl.dropFirstWobbleGoal);
                    telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                    telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                    telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutoControl.pickAndDropSecondWobbleGoal);
                    break;
                }
                if (hzGamepad.getButtonXPress()) {
                    hzAutoControl.dropFirstWobbleGoal = true;
                    hzAutoControl.pickRingFromTargetMarker = true;
                    hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = true;
                    hzAutoControl.pickAndDropSecondWobbleGoal = false;
                    telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutoControl.dropFirstWobbleGoal);
                    telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                    telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                    telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutoControl.pickAndDropSecondWobbleGoal);
                    break;
                }
                if (hzGamepad.getButtonBPress()) {
                    hzAutoControl.dropFirstWobbleGoal = true;
                    hzAutoControl.pickRingFromTargetMarker = true;
                    hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = true;
                    hzAutoControl.pickAndDropSecondWobbleGoal = true;
                    telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutoControl.dropFirstWobbleGoal);
                    telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                    telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                    telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutoControl.pickAndDropSecondWobbleGoal);
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

        telemetry.addData("GameField.playingAlliance : ", HzGameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", HzGameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", HzGameField.currentPose);
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
        switch (hzMagazine.getMagazinePosition()) {
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
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazine.magazineLaunchTouchSensor.isPressed());
        //telemetry.addData("hzMagazine.moveMagazineToCollectState",hzMagazine.moveMagazineToCollectState);
        telemetry.addData("magazineCollectTouchSensor.getState():", hzMagazine.magazineCollectTouchSensor.isPressed());


        //********** Intake Debug *******
        switch (hzIntake.getIntakeState()){
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
        telemetry.addData("hzLaunchController.launchMode : ", hzLaunchController.launchMode);
        telemetry.addData("hzLaunchController.deactivateLaunchReadinessState :",hzLaunchController.deactivateLaunchReadinessState);
        telemetry.addData("hzLaunchController.activateLaunchReadinessState :",hzLaunchController.activateLaunchReadinessState);
        telemetry.addData("hzLaunchController.launchReadiness : ", hzLaunchController.launchReadiness);
        telemetry.addData("hzLaunchController.launchActivation : ", hzLaunchController.launchActivation);
        telemetry.addData("hzLaunchController.lcTarget : ", hzLaunchController.lcTarget);
        telemetry.addData("hzLaunchController.lcTargetVector", hzLaunchController.lcTargetVector);
        telemetry.addData("hzLaunchController.distanceFromTarget : ", hzLaunchController.distanceFromTarget);
        telemetry.addData("hzLauncher.launcherFlyWheelMotor.getPower() : ", hzLauncher.launcherFlyWheelMotor.getPower());
        telemetry.addData("hzLaunchController.lclaunchMotorVelocity : ", hzLaunchController.lclaunchMotorVelocity);
        telemetry.addData("hzLauncher.flyWheelVelocityPowerShot : ", hzLauncher.flyWheelVelocityPowerShot);
        telemetry.addData("hzLauncher.flyWheelVelocityHighGoal : ", hzLauncher.flyWheelVelocityHighGoal);
        telemetry.addData("hzLauncher.launcherFlyWheelMotor.getVelocity() : ", hzLauncher.launcherFlyWheelMotor.getVelocity());
        telemetry.addData("hzDrive.drivePointToAlign : ", hzDrive.drivePointToAlign);

        //******* Launcher Debug *********
        //telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        telemetry.addData("launcherRingPlungerServo.getPosition() : ", hzLauncher.launcherRingPlungerServo.getPosition());

        switch (hzLauncher.getLauncherState()){
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
        telemetry.addData("armMotor.baseline", hzArm.baselineEncoderCount);
        telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());
        telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());

        switch (hzArm.getGripServoState()){
            case OPENED  : {
                telemetry.addData("hzArm.getGripServoState()", "OPENED");
                break;
            }
            case CLOSED: {
                telemetry.addData("hzArm.getGripServoState()", "CLOSED");
                break;
            }
        }

        switch (hzArm.getCurrentArmPosition()){
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

