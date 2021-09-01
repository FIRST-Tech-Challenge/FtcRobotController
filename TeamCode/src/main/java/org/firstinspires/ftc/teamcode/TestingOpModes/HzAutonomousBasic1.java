package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzArmUltimateGoal;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzAutonomousControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.GameOpModes.Examples.HzGameFieldUltimateGoal;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzGamepadControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzIntakeUltimateGoal;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzLaunchSubControllerUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzLauncherUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzMagazineUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzVuforiaUltimateGoal;

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

    double turnAnglePowershot12 = Math.toRadians(-5);
    double turnAnglePowershot23 = Math.toRadians(-5);
    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        hzDrive = new HzDrive(hardwareMap);
        hzMagazineUltimateGoal = new HzMagazineUltimateGoal(hardwareMap);
        hzIntakeUltimateGoal = new HzIntakeUltimateGoal(hardwareMap);

        hzLauncherUltimateGoal = new HzLauncherUltimateGoal(hardwareMap);
        hzArmUltimateGoal = new HzArmUltimateGoal(hardwareMap);
        hzLaunchSubControllerUltimateGoal = new HzLaunchSubControllerUltimateGoal(hardwareMap, hzLauncherUltimateGoal, hzIntakeUltimateGoal, hzMagazineUltimateGoal, hzDrive);
        hzGamepadControllerUltimateGoal = new HzGamepadControllerUltimateGoal(gamepad1,hzDrive, hzMagazineUltimateGoal, hzIntakeUltimateGoal, hzLaunchSubControllerUltimateGoal, hzLauncherUltimateGoal, hzArmUltimateGoal);
        hzAutonomousController = new HzAutonomousControllerUltimateGoal(hzDrive, hzMagazineUltimateGoal, hzIntakeUltimateGoal, hzLaunchSubControllerUltimateGoal, hzLauncherUltimateGoal, hzArmUltimateGoal);

        //initialConfiguration();
        selectGamePlan();
        hzVuforiaUltimateGoal = new HzVuforiaUltimateGoal(hardwareMap, activeWebcam);

        // Initiate Camera even before Start is pressed.
        //waitForStart();

        hzVuforiaUltimateGoal.activateVuforiaTensorFlow();

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        hzIntakeUltimateGoal.setIntakeReleaseHold();
        hzAutonomousController.setMagazineToLaunch();

        while (hzMagazineUltimateGoal.magazineLaunchTouchSensor.isPressed() == false) {
            hzAutonomousController.setMagazineToLaunch();
            if (isStopRequested()) return;
        }

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Init is pressed at this time, and start is not pressed yet

            //Run Vuforia Tensor Flow
            targetZone = hzVuforiaUltimateGoal.runVuforiaTensorFlow();

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
            hzLaunchSubControllerUltimateGoal.launchMode = HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.MANUAL;

            while (opModeIsActive() && !isStopRequested() && !parked) {

                hzVuforiaUltimateGoal.deactivateVuforiaTensorFlow();

                if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    if (startPose == HzGameFieldUltimateGoal.BLUE_INNER_START_LINE ) {
                        autonomousStarted = true;
                        runAutoBlueInner();
                    } else if (startPose == HzGameFieldUltimateGoal.BLUE_OUTER_START_LINE ) {
                        autonomousStarted = true;
                        runAutoBlueOuterHighGoal();
                    }
                } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                    if (startPose == HzGameFieldUltimateGoal.RED_INNER_START_LINE ) {
                        autonomousStarted = true;
                        //runAutoRedInnerHighGoal();
                    } else if (startPose == HzGameFieldUltimateGoal.RED_OUTER_START_LINE ) {
                        autonomousStarted = true;
                        //runAutoRedOuterHighGoal();
                    }
                }

                hzIntakeUltimateGoal.setIntakeReleaseOpen();

                //Move to Launching Position
                parked = true;
                HzGameFieldUltimateGoal.currentPose = hzDrive.getPoseEstimate();
                HzGameFieldUltimateGoal.poseSetInAutonomous = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        HzGameFieldUltimateGoal.currentPose = hzDrive.getPoseEstimate();
        HzGameFieldUltimateGoal.poseSetInAutonomous = true;
    }

    public void runAutoBlueInner(){

        // Set magazine to Launch in case it slipped
        hzAutonomousController.setMagazineToLaunch();

        // Move to launch position and launch rings to High Goal or Powershots
        if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
            hzAutonomousController.setLaunchTargetHighGoal();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-40,6,Math.toRadians(45)))
                    .build();
            hzDrive.followTrajectory(traj);
            hzAutonomousController.setMagazineToLaunch();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,14,Math.toRadians(18)))
                    .build();
            hzDrive.followTrajectory(traj);

            launch3RingsToHighGoal();
        } else {
            hzAutonomousController.setLaunchTargetPowerShot1();
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

        if ((hzAutonomousController.pickRingFromTargetMarker == false) || (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A)){ // Park
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
            hzIntakeUltimateGoal.setIntakeReleaseOpen();
            hzAutonomousController.setIntakeStart();
            hzWait(500);

            if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.B) {
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
            if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
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

            if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal) {
                hzAutonomousController.setIntakeStop();
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();

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
                hzAutonomousController.setIntakeStop();
            }
        }
    }

    public void runAutoBlueOuterHighGoal(){
        // Set magazine to Launch in case it slipped
        hzAutonomousController.setMagazineToLaunch();

        // Move to launch position and launch rings to High Goal or Powershots
        if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
            hzAutonomousController.setLaunchTargetHighGoal();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-40,50,Math.toRadians(-45)))
                    .build();
            hzDrive.followTrajectory(traj);
            hzAutonomousController.setMagazineToLaunch();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,50,Math.toRadians(-10)))
                    .build();
            hzDrive.followTrajectory(traj);

            launch3RingsToHighGoal();
        } else {
            hzAutonomousController.setLaunchTargetPowerShot1();
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

        if ((hzAutonomousController.pickRingFromTargetMarker == false) || (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A)){ // Park
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
            hzIntakeUltimateGoal.setIntakeReleaseOpen();
            hzAutonomousController.setIntakeStart();
            hzWait(500);

            if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.B) {
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
            if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
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

            if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal) {
                hzAutonomousController.setIntakeStop();
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();

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
                hzAutonomousController.setIntakeStop();
            }
        }
    }

    public void launch3RingsToHighGoal(){
        hzAutonomousController.setMagazineToLaunch();
        hzWait(500);
        hzAutonomousController.setLaunchTargetHighGoal();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();
        hzWait(400);

        hzAutonomousController.setLaunchTargetOff();
        hzAutonomousController.setMagazineToCollect();
    }

    public void launch3RingsToPowerShots(){
        hzAutonomousController.setMagazineToLaunch();
        hzWait(500);
        hzAutonomousController.setLaunchTargetPowerShot1();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(400);
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

        hzAutonomousController.setLaunchTargetOff();
        hzAutonomousController.setMagazineToCollect();

    }

    public void dropWobbleGoalInTarget(){
        hzAutonomousController.setMoveArmDropWobbleAutonoumous();
        hzWait(1000);
        hzAutonomousController.runOpenGrip();
        hzWait(300);
        hzAutonomousController.setMoveArmHoldUpWobbleRing();
        hzAutonomousController.setMoveArmParked();
    }


    public void hzWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            hzAutonomousController.runAutoControl();
        }
    }


    public void initialConfiguration(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "6:50 : 12/05");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Red:B, Blue:X)");
        telemetry.update();

        while (!isStopRequested()) {
            if (hzGamepadControllerUltimateGoal.getButtonBPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE;
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
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    startPose = HzGameFieldUltimateGoal.RED_INNER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    startPose = HzGameFieldUltimateGoal.RED_OUTER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    startPose = HzGameFieldUltimateGoal.BLUE_INNER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    startPose = HzGameFieldUltimateGoal.BLUE_OUTER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
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
            if (hzGamepadControllerUltimateGoal.getButtonBPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE;
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
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    startPose = HzGameFieldUltimateGoal.RED_INNER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    startPose = HzGameFieldUltimateGoal.RED_OUTER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameFieldUltimateGoal.playingAlliance == HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                    startPose = HzGameFieldUltimateGoal.BLUE_INNER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                    startPose = HzGameFieldUltimateGoal.BLUE_OUTER_START_LINE;
                    activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            //telemetry.addData("10s Timer : Default Pose : BLUE_INNER_START_LINE : %.3f", timer.time());
            telemetry.update();
        }
        telemetry.update();


        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameFieldUltimateGoal.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ",activeWebcam);
            telemetry.addLine();
            telemetry.addData("Please select launch target : ", "(X) for Powershot, (B) for High Goal, ");
            if (hzGamepadControllerUltimateGoal.getButtonBPress()) {
                hzAutonomousController.autoLaunchAim = HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutonomousController.autoLaunchAim);
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                hzAutonomousController.autoLaunchAim = HzAutonomousControllerUltimateGoal.AutoLaunchAim.POWERSHOT;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutonomousController.autoLaunchAim);
                break;
            }
            telemetry.update();
            hzWait(200);
        }

        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameFieldUltimateGoal.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ", activeWebcam);
            telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutonomousController.autoLaunchAim);
            telemetry.addLine();
            telemetry.addData("Please select option for Picking rings from target markers and launching : ", "");
            telemetry.addData("    (X):","Pick rings and launch after Wobble Goal, and park");
            telemetry.addData("    (Y):","Pick rings after Wobble Goal and park (No launching)");
            telemetry.addData("    (A):","Park after Wobble Goal (No ring Pick)");

            if (hzGamepadControllerUltimateGoal.getButtonAPress()) {
                hzAutonomousController.pickRingFromTargetMarker = false;
                hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = false;
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutonomousController.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal);
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonYPress()) {
                hzAutonomousController.pickRingFromTargetMarker = true;
                hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = false;
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutonomousController.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal);
                break;
            }
            if (hzGamepadControllerUltimateGoal.getButtonXPress()) {
                hzAutonomousController.pickRingFromTargetMarker = true;
                hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal = true;
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutonomousController.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal);
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

        telemetry.addData("GameField.playingAlliance : ", HzGameFieldUltimateGoal.playingAlliance);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", hzDrive.driveMode);
        telemetry.addData("PoseEstimate :", hzDrive.poseEstimate);
        telemetry.addData("HzGameField.currentPose", HzGameFieldUltimateGoal.currentPose);

        //telemetry.addData("Visible Target : ", hzVuforia.visibleTargetName);
        telemetry.addData("hzVuforia.targetZoneDetected", hzVuforiaUltimateGoal.targetZoneDetected);
        telemetry.addData("targetZone :", targetZone);
        // Print pose to telemetry
        //telemetry.addData("PoseVuforia :",hzVuforia1.poseVuforia);

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
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazineUltimateGoal.magazineLaunchTouchSensor.isPressed());
        telemetry.addData("magazineCollectTouchSensor.getState():", hzMagazineUltimateGoal.magazineCollectTouchSensor.isPressed());

        //********** Intake Debug *******
        //telemetry.addData("hzGamepad1.getDpad_downPress()", hzGamepad.getDpad_downPress());
        //telemetry.addData("hzGamepad1.getDpad_upPress()", hzGamepad.getDpad_upPress());
        //telemetry.addData("intakeMotor.isBusy()", hzIntake.intakeMotor.isBusy());
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
        telemetry.addData("hzLaunchController.launchReadiness : ", hzLaunchSubControllerUltimateGoal.launchReadiness);
        telemetry.addData("hzLaunchController.launchActivation : ", hzLaunchSubControllerUltimateGoal.launchActivation);
        telemetry.addData("hzLaunchController.lcTarget : ", hzLaunchSubControllerUltimateGoal.lcTarget);
        telemetry.addData("hzLaunchController.lcTargetVector", hzLaunchSubControllerUltimateGoal.lcTargetVector);
        telemetry.addData("hzLaunchController.distanceFromTarget : ", hzLaunchSubControllerUltimateGoal.distanceFromTarget);
        telemetry.addData("hzLaunchController.lclaunchMotorPower : ", hzLaunchSubControllerUltimateGoal.lclaunchMotorPower);
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
        //telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());
        //telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());

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

        //telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());
        //telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());

        //telemetry.addData("armGripServo.getCurrentPosition()", hzArm.armGripServo.getPosition());
        //telemetry.addData("hzGamepad.getLeftTriggerPress()", hzGamepad.getLeftTriggerPress());

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
