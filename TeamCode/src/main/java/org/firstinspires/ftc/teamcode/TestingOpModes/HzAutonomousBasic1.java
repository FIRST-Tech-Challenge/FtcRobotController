package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 *
 * setAutoMoveArmPickWobble()
 * setAutoMoveArmDropWobbleRing()
 *
 */
@Autonomous(name = "Hazmat Autonomous 1", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp RR")
@Disabled
public class HzAutonomousBasic1 extends LinearOpMode {

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

    double turnAnglePowershot12 = Math.toRadians(-5);
    double turnAnglePowershot23 = Math.toRadians(-5);
    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        hzDrive = new HzDrive(hardwareMap);
        hzMagazine = new HzMagazine(hardwareMap);
        hzIntake = new HzIntake(hardwareMap);

        hzLauncher = new HzLauncher(hardwareMap);
        hzArm = new HzArm(hardwareMap);
        hzLaunchController = new HzLaunchController(hardwareMap, hzLauncher, hzIntake, hzMagazine, hzDrive);
        hzGamepad = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);
        hzAutoControl = new HzAutoControl(hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        //initialConfiguration();
        selectGamePlan();
        hzVuforia = new HzVuforia(hardwareMap, activeWebcam);

        // Initiate Camera even before Start is pressed.
        //waitForStart();

        hzVuforia.activateVuforiaTensorFlow();

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        hzIntake.setIntakeReleaseHold();
        hzAutoControl.setMagazineToLaunch();

        while (hzMagazine.magazineLaunchTouchSensor.isPressed() == false) {
            hzAutoControl.setMagazineToLaunch();
            if (isStopRequested()) return;
        }

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Init is pressed at this time, and start is not pressed yet

            //Run Vuforia Tensor Flow
            targetZone = hzVuforia.runVuforiaTensorFlow();

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
            hzLaunchController.launchMode = HzLaunchController.LAUNCH_MODE.MANUAL;

            while (opModeIsActive() && !isStopRequested() && !parked) {

                hzVuforia.deactivateVuforiaTensorFlow();

                if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    if (startPose == HzGameField.BLUE_INNER_START_LINE ) {
                        autonomousStarted = true;
                        runAutoBlueInner();
                    } else if (startPose == HzGameField.BLUE_OUTER_START_LINE ) {
                        autonomousStarted = true;
                        runAutoBlueOuterHighGoal();
                    }
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    if (startPose == HzGameField.RED_INNER_START_LINE ) {
                        autonomousStarted = true;
                        //runAutoRedInnerHighGoal();
                    } else if (startPose == HzGameField.RED_OUTER_START_LINE ) {
                        autonomousStarted = true;
                        //runAutoRedOuterHighGoal();
                    }
                }

                hzIntake.setIntakeReleaseOpen();

                //Move to Launching Position
                parked = true;
                HzGameField.currentPose = hzDrive.getPoseEstimate();
                HzGameField.poseSetInAutonomous = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        HzGameField.currentPose = hzDrive.getPoseEstimate();
        HzGameField.poseSetInAutonomous = true;
    }

    public void runAutoBlueInner(){

        // Set magazine to Launch in case it slipped
        hzAutoControl.setMagazineToLaunch();

        // Move to launch position and launch rings to High Goal or Powershots
        if (hzAutoControl.autoLaunchAim == HzAutoControl.AutoLaunchAim.HIGHGOAL) {
            hzAutoControl.setLaunchTargetHighGoal();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-40,6,Math.toRadians(45)))
                    .build();
            hzDrive.followTrajectory(traj);
            hzAutoControl.setMagazineToLaunch();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,14,Math.toRadians(18)))
                    .build();
            hzDrive.followTrajectory(traj);

            launch3RingsToHighGoal();
        } else {
            hzAutoControl.setLaunchTargetPowerShot1();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-40,6,Math.toRadians(45)))
                    .build();
            hzDrive.followTrajectory(traj);
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-10,14,Math.toRadians(5)),Math.toRadians(0))
                    .build();
            hzDrive.followTrajectory(traj);

            //Set turn angles prior to launching
            turnAnglePowershot12 = Math.toRadians(-5);
            turnAnglePowershot23 = Math.toRadians(-5);
            launch3RingsToPowerShots();
        }

        // Move to drop wobble goal on target
        switch (targetZone){
            case A:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(46,22,Math.toRadians(-45)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(27,43,Math.toRadians(-45)))
                        .build();
                hzDrive.followTrajectory(traj);

                break;

            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(46,22,Math.toRadians(-45)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(52,43,Math.toRadians(-90)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;

        }

        dropWobbleGoalInTarget();

        if ((hzAutoControl.pickRingFromTargetMarker == false) || (targetZone == HzGameField.TARGET_ZONE.A)){ // Park
            //Park
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(50,16,Math.toRadians(-45)))
                    .build();
            hzDrive.followTrajectory(traj);

            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, 20, Math.toRadians(0)))
                    .build();
            hzDrive.followTrajectory(traj);

            /* Alternate
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(10, 0, Math.toRadians(0)))
                    .build();
            hzDrive.followTrajectory(traj);
            */
            return;
        } else { //Target Zone is B or C and pickRingFromTargetMarker == True
            //Pick rings from Target Marker
            hzIntake.setIntakeReleaseOpen();
            hzAutoControl.setIntakeStart();
            hzWait(500);

            if (targetZone == HzGameField.TARGET_ZONE.B) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10,33,Math.toRadians(135)))
                        .build();
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22,33,Math.toRadians(135)))
                        .build();
                hzDrive.followTrajectory(traj);

                hzWait(300);
            }

            // Spline to (24,24,0)
            if (targetZone == HzGameField.TARGET_ZONE.C) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10,33,Math.toRadians(-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22,33,Math.toRadians(-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-33, 33, Math.toRadians(-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                hzWait(300);
            }

            if (hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal) {
                hzAutoControl.setIntakeStop();
                hzAutoControl.setMagazineToLaunch();
                hzAutoControl.setLaunchTargetHighGoal();

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-13, 33, Math.toRadians(0)))
                        .build();
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();

                //Park
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, 33, Math.toRadians(0)))
                        .build();
                hzDrive.followTrajectory(traj);
            } else { //Move to baseline and park in safe zone
                traj =  hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-50,16,Math.toRadians(-90)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(10, 0, Math.toRadians(0)))
                        .build();
                hzDrive.followTrajectory(traj);
                hzAutoControl.setIntakeStop();
            }
        }
    }

    public void runAutoBlueOuterHighGoal(){
        // Set magazine to Launch in case it slipped
        hzAutoControl.setMagazineToLaunch();

        // Move to launch position and launch rings to High Goal or Powershots
        if (hzAutoControl.autoLaunchAim == HzAutoControl.AutoLaunchAim.HIGHGOAL) {
            hzAutoControl.setLaunchTargetHighGoal();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-40,50,Math.toRadians(-45)))
                    .build();
            hzDrive.followTrajectory(traj);
            hzAutoControl.setMagazineToLaunch();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,50,Math.toRadians(-10)))
                    .build();
            hzDrive.followTrajectory(traj);

            launch3RingsToHighGoal();
        } else {
            hzAutoControl.setLaunchTargetPowerShot1();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-40,50,Math.toRadians(45)))
                    .build();
            hzDrive.followTrajectory(traj);
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-10,50,Math.toRadians(-15)),Math.toRadians(0))
                    .build();
            hzDrive.followTrajectory(traj);

            //Set turn angles prior to launching
            turnAnglePowershot12 = Math.toRadians(-5);
            turnAnglePowershot23 = Math.toRadians(-5);
            launch3RingsToPowerShots();
        }


        switch (targetZone){
            case A:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10,48,Math.toRadians(-135)))
                        .build();
                hzDrive.followTrajectory(traj);
                //Consider just turn(Math.toRadians(-120);
                break;
            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(15,48,Math.toRadians(135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(35,50,Math.toRadians(-135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;

        }

        dropWobbleGoalInTarget();

        if ((hzAutoControl.pickRingFromTargetMarker == false) || (targetZone == HzGameField.TARGET_ZONE.A)){ // Park
            //Park
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, 20, Math.toRadians(0)))
                    .build();
            hzDrive.followTrajectory(traj);

            /* Alternate
            traj =  hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-50,16,Math.toRadians(-90)))
                    .build();
            hzDrive.followTrajectory(traj);
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(10, 0, Math.toRadians(0)))
                    .build();
            hzDrive.followTrajectory(traj);
            */
            return;

        } else { //Target Zone is B or C and pickRingFromTargetMarker == True
            //Pick rings from Target Marker
            hzIntake.setIntakeReleaseOpen();
            hzAutoControl.setIntakeStart();
            hzWait(500);

            if (targetZone == HzGameField.TARGET_ZONE.B) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10,33,Math.toRadians(135)))
                        .build();
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22,33,Math.toRadians(135)))
                        .build();
                hzDrive.followTrajectory(traj);

                hzWait(300);
            }

            // Spline to (24,24,0)
            if (targetZone == HzGameField.TARGET_ZONE.C) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10,33,Math.toRadians(-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22,33,Math.toRadians(-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-33, 33, Math.toRadians(-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                hzWait(300);
            }

            if (hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal) {
                hzAutoControl.setIntakeStop();
                hzAutoControl.setMagazineToLaunch();
                hzAutoControl.setLaunchTargetHighGoal();

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-13, 33, Math.toRadians(0)))
                        .build();
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();

                //Park
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, 33, Math.toRadians(0)))
                        .build();
                hzDrive.followTrajectory(traj);
            } else { //Move to baseline and park in safe zone
                traj =  hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-50,16,Math.toRadians(-90)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(10, 0, Math.toRadians(0)))
                        .build();
                hzDrive.followTrajectory(traj);
                hzAutoControl.setIntakeStop();
            }
        }
    }

    public void launch3RingsToHighGoal(){
        hzAutoControl.setMagazineToLaunch();
        hzWait(500);
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setMagazineToLaunch();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setMagazineToLaunch();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setMagazineToLaunch();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();
        hzWait(400);

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();
    }

    public void launch3RingsToPowerShots(){
        hzAutoControl.setMagazineToLaunch();
        hzWait(500);
        hzAutoControl.setLaunchTargetPowerShot1();
        hzAutoControl.setMagazineToLaunch();
        hzWait(400);
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

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();

    }

    public void dropWobbleGoalInTarget(){
        hzAutoControl.setMoveArmDropWobbleAutonoumous();
        hzWait(1000);
        hzAutoControl.runOpenGrip();
        hzWait(300);
        hzAutoControl.setMoveArmHoldUpWobbleRing();
        hzAutoControl.setMoveArmParked();
    }


    public void hzWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            hzAutoControl.runAutoControl();
        }
    }


    public void initialConfiguration(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "6:50 : 12/05");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Red:B, Blue:X)");
        telemetry.update();

        while (!isStopRequested()) {
            if (hzGamepad.getButtonBPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            //telemetry.addData("10s time : Default Alliance A :",);
            telemetry.update();
        }

        telemetry.update();
        sleep(500);

        //***** Select Start Pose ******
        //timer.reset();
        telemetry.addData("Enter Start Pose :", "(Inner:A, Outer:Y)");
        while (!isStopRequested()) {
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    startPose = HzGameField.RED_INNER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonYPress()) {
                    startPose = HzGameField.RED_OUTER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    startPose = HzGameField.BLUE_INNER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonYPress()) {
                    startPose = HzGameField.BLUE_OUTER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            //telemetry.addData("10s Timer : Default Pose : BLUE_INNER_START_LINE : %.3f", timer.time());
            telemetry.update();
        }
        telemetry.update();
        //sleep(500);
    }

    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "4:11 :: 1/14");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Blue: (X),    Red: (B))");
        telemetry.update();

        /*HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
        startPose = HzGameField.BLUE_INNER_START_LINE;
        activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;*/

        while (!isStopRequested()) {
            if (hzGamepad.getButtonBPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }

        telemetry.update();
        hzWait(200);

        //***** Select Start Pose ******
        //timer.reset();
        telemetry.addData("Enter Start Pose :", "(Inner: (A) ,    Outer: (Y))");
        while (!isStopRequested()) {
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    startPose = HzGameField.RED_INNER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonYPress()) {
                    startPose = HzGameField.RED_OUTER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    startPose = HzGameField.BLUE_INNER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonYPress()) {
                    startPose = HzGameField.BLUE_OUTER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            //telemetry.addData("10s Timer : Default Pose : BLUE_INNER_START_LINE : %.3f", timer.time());
            telemetry.update();
        }
        telemetry.update();


        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameField.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ",activeWebcam);
            telemetry.addLine();
            telemetry.addData("Please select launch target : ", "(X) for Powershot, (B) for High Goal, ");
            if (hzGamepad.getButtonBPress()) {
                hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.POWERSHOT;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                break;
            }
            telemetry.update();
            hzWait(200);
        }

        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameField.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ", activeWebcam);
            telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
            telemetry.addLine();
            telemetry.addData("Please select option for Picking rings from target markers and launching : ", "");
            telemetry.addData("    (X):","Pick rings and launch after Wobble Goal, and park");
            telemetry.addData("    (Y):","Pick rings after Wobble Goal and park (No launching)");
            telemetry.addData("    (A):","Park after Wobble Goal (No ring Pick)");

            if (hzGamepad.getButtonAPress()) {
                hzAutoControl.pickRingFromTargetMarker = false;
                hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = false;
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                break;
            }
            if (hzGamepad.getButtonYPress()) {
                hzAutoControl.pickRingFromTargetMarker = true;
                hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = false;
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                hzAutoControl.pickRingFromTargetMarker = true;
                hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = true;
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                break;
            }
            telemetry.update();
        }

        telemetry.update();
        sleep(200);
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);
        telemetry.addData("autonomousStarted : ", autonomousStarted);

        telemetry.addData("GameField.playingAlliance : ", HzGameField.playingAlliance);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", hzDrive.driveMode);
        telemetry.addData("PoseEstimate :", hzDrive.poseEstimate);
        telemetry.addData("HzGameField.currentPose",HzGameField.currentPose);

        //telemetry.addData("Visible Target : ", hzVuforia.visibleTargetName);
        telemetry.addData("hzVuforia.targetZoneDetected", hzVuforia.targetZoneDetected);
        telemetry.addData("targetZone :", targetZone);
        // Print pose to telemetry
        //telemetry.addData("PoseVuforia :",hzVuforia1.poseVuforia);

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
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazine.magazineLaunchTouchSensor.isPressed());
        telemetry.addData("magazineCollectTouchSensor.getState():", hzMagazine.magazineCollectTouchSensor.isPressed());

        //********** Intake Debug *******
        //telemetry.addData("hzGamepad1.getDpad_downPress()", hzGamepad.getDpad_downPress());
        //telemetry.addData("hzGamepad1.getDpad_upPress()", hzGamepad.getDpad_upPress());
        //telemetry.addData("intakeMotor.isBusy()", hzIntake.intakeMotor.isBusy());
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
        telemetry.addData("hzLaunchController.launchReadiness : ", hzLaunchController.launchReadiness);
        telemetry.addData("hzLaunchController.launchActivation : ", hzLaunchController.launchActivation);
        telemetry.addData("hzLaunchController.lcTarget : ", hzLaunchController.lcTarget);
        telemetry.addData("hzLaunchController.lcTargetVector", hzLaunchController.lcTargetVector);
        telemetry.addData("hzLaunchController.distanceFromTarget : ", hzLaunchController.distanceFromTarget);
        telemetry.addData("hzLaunchController.lclaunchMotorPower : ", hzLaunchController.lclaunchMotorPower);
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
        //telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());
        //telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());

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

        //telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());
        //telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());

        //telemetry.addData("armGripServo.getCurrentPosition()", hzArm.armGripServo.getPosition());
        //telemetry.addData("hzGamepad.getLeftTriggerPress()", hzGamepad.getLeftTriggerPress());

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

/*
Turn command : drive.turn(Math.toRadians(ANGLE));

Blue Alliance - Inner position-Target A
    Start Pose : (TBD, 25, ~-70deg)
    Spline to (0,12,0)
    Launch ring to power shot 1,2,3, (automatic alignment)
    Spline to (12,36,-90)
    Drop wobble goal
    Spline to (24,24,0)

Blue Alliance - Inner position-Target B
    Start Pose : (TBD, 25, ~-70deg)
    Launch ring to power shot 1,2,3, (automatic alignment)
    spline to (12,36,180)
    Drop wobble goal
    Turn intake on
    Straight to (-24,36,180)
    Turn 180 deg
    Launch to high goal
    Move to (12,36,0)

Blue Alliance - Inner position-Target C - Option 1
    Start Pose : (TBD, 25, ~-70deg)
    Launch ring to power shot 1,2,3, (automatic alignment)
    Turn  to 145 deg
    Turn Intake on
    Straight to (-24,36,145)
    Turn to 0
    Launch to High goal x 3
    Spline to (40,40,-145)
    Drop Wobble goal
    Spline to (24,24,0)

Blue Alliance - Inner position-Target C - Option 2
    Start Pose : (TBD, 25, ~-70deg)
    Launch ring to power shot 1,2,3, (automatic alignment)
    Spline to (40,40,-145)
    Drop Wobble goal
    Turn Intake on
    Spline to (-24,36,180)
    Turn to 0
    Launch to High goal x 3
    Spline to (24,24,0)


 */
