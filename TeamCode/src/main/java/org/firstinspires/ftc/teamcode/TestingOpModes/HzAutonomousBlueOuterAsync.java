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
@Autonomous(name = "Hazmat Blue Outer Autonomous", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp RR")
@Disabled
public class HzAutonomousBlueOuterAsync extends LinearOpMode {

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

        selectGamePlan();
        //HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
        //startPose = HzGameField.BLUE_OUTER_START_LINE;
        //activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
        //hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.HIGHGOAL;
        //hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.POWERSHOT;
        //hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = true;

        hzVuforiaUltimateGoal = new HzVuforiaUltimateGoal(hardwareMap, activeWebcam);

        // Initiate Camera even before Start is pressed.
        //waitForStart();

        hzVuforiaUltimateGoal.activateVuforiaTensorFlow();
        boolean vuforiaTensorFlowActivated = true;

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        hzIntakeUltimateGoal.setIntakeReleaseHold();
        hzAutonomousController.setMagazineToLaunch();

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

            buildAutoBlueOuterTrajectory();


            while (opModeIsActive() && !parked) {
                if (vuforiaTensorFlowActivated == true) {
                    hzVuforiaUltimateGoal.deactivateVuforiaTensorFlow();
                    vuforiaTensorFlowActivated = false;
                }
                autonomousStarted = true;
                runAutoBlueOuter();
                hzDrive.update();

                //Move to Launching Positio
                HzGameFieldUltimateGoal.currentPose = hzDrive.getPoseEstimate();
                HzGameFieldUltimateGoal.poseSetInAutonomous = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        hzIntakeUltimateGoal.setIntakeReleaseOpen();

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        HzGameFieldUltimateGoal.currentPose = hzDrive.getPoseEstimate();
        HzGameFieldUltimateGoal.poseSetInAutonomous = true;
    }

    enum AutoStepState {
        INITIATE,
        TRAJ_LAUNCH_RINGS_HIGHGOAL,
        LAUNCH_RINGS_HIGHGOAL,
        TRAJ_LAUNCH_RING_POWERSHOT1,
        LAUNCH_RINGS_POWERSHOT1,
        TRAJ_LAUNCH_RING_POWERSHOT2,
        LAUNCH_RINGS_POWERSHOT2,
        TRAJ_LAUNCH_RING_POWERSHOT3,
        LAUNCH_RINGS_POWERSHOT3,
        //TRAJ_SAFE_POSITION_BEFORE_WOBBLE_TARGET_DROP,
        TRAJ_WOBBLE_DROP_POSITION,
        DROP_WOBBLE_GOAL_ON_TARGET,
        TRAJ_SAFE_POSITION_AFTER_WOBBLE_TARGET_DROP,
        TRAJ_PARK_WITHOUT_PICK_RING,
        //TRAJ_SAFE_POSITION_BEFORE_PICK_RINGS,
        START_INTAKE,
        TRAJ_PICK_RINGS_FROM_TARGET_MARK,
        TRAJ_TURN_TO_HIGH_GOAL,
        LAUNCH_RINGS_HIGHGOAL_SECOND,
        TRAJ_SAFE_POSITION_BEFORE_PARK,
        TRAJ_PARK,
        END,
        IDLE
    }
    AutoStepState currentAutoStepState = AutoStepState.INITIATE;

    Trajectory trajLaunchRingsHighGoal;
    Trajectory trajLaunchRingPowershot1;
    //Trajectory trajLaunchRingPowershot2;
    //Trajectory trajLaunchRingPowershot3;
    //Trajectory trajSafePositionBeforeWobbleTargetDrop;
    Trajectory trajWobbleDropPosition;
    Trajectory trajSafePositionAfterWobbleTargetDrop;
    Trajectory trajParkWithoutPickRing;
    //Trajectory trajSafe_Position_Before_Pick_Rings;
    Trajectory trajPickRingsFromTargetMark;
    Trajectory trajTurnToHighGoal;
    Trajectory trajSafePostionBeforePark;
    Trajectory trajPark;

    Pose2d lastPose = startPose;
    double tuneAnglePowershot12 = Math.toRadians(-10);
    double tuneAnglePowershot23 = Math.toRadians(-10);


    public void buildAutoBlueOuterTrajectory(){
        if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
            trajLaunchRingsHighGoal = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-10, 50, Math.toRadians(-10)))
                    .build();
            lastPose = trajLaunchRingsHighGoal.end();
        } else { //hzAutoControl.autoLaunchAim == HzAutoControl.AutoLaunchAim.POWERSHOT
            trajLaunchRingPowershot1 = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-10, 50, Math.toRadians(-20)))
                    .build();
            lastPose = trajLaunchRingPowershot1.end();
            //Turn to Powershot2
            lastPose = lastPose.plus(new Pose2d(0, 0, tuneAnglePowershot12));
            //Turn to Powershot3
            lastPose = lastPose.plus(new Pose2d(0, 0, tuneAnglePowershot23));
        }

        switch (targetZone){
            case A:
                trajWobbleDropPosition = hzDrive.trajectoryBuilder(lastPose)
                        .lineToSplineHeading(new Pose2d(-10,48,Math.toRadians(-135)))
                        .build();
                lastPose = trajWobbleDropPosition.end();
                /*trajPickRingsFromTargetMark = hzDrive.trajectoryBuilder(lastPose)
                        .lineToLinearHeading(new Pose2d(-5,36,Math.toRadians(-180)))
                        .build();
                lastPose = trajPickRingsFromTargetMark.end();*/
                trajSafePositionAfterWobbleTargetDrop = hzDrive.trajectoryBuilder(lastPose)
                        .lineToSplineHeading(new Pose2d(36, 36, Math.toRadians(0)))
                        .build();
                lastPose = trajSafePositionAfterWobbleTargetDrop.end();
                trajParkWithoutPickRing = hzDrive.trajectoryBuilder(lastPose)
                        .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(0)))
                        .build();
                lastPose = trajParkWithoutPickRing.end();
                break;
            case B:
                trajWobbleDropPosition = hzDrive.trajectoryBuilder(lastPose)
                        .lineToSplineHeading(new Pose2d(15,48,Math.toRadians(135)))
                        .build();
                lastPose = trajWobbleDropPosition.end();
                if (hzAutonomousController.pickRingFromTargetMarker == true) {
                    trajPickRingsFromTargetMark = hzDrive.trajectoryBuilder(lastPose)
                            .lineToLinearHeading(new Pose2d(-5, 36, Math.toRadians(-180)))
                            .lineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(-180)))
                            .build();
                    lastPose = trajPickRingsFromTargetMark.end();
                    if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal == true) {
                        trajTurnToHighGoal = hzDrive.trajectoryBuilder(lastPose)
                                .lineToLinearHeading(new Pose2d(-5, 36, Math.toRadians(0)))
                                .build();
                        lastPose = trajTurnToHighGoal.end();
                    }
                    trajSafePostionBeforePark =  hzDrive.trajectoryBuilder(lastPose)
                            .lineToLinearHeading(new Pose2d(-48,22,Math.toRadians(-90)))
                            .build();
                    lastPose = trajSafePostionBeforePark.end();
                    trajPark = hzDrive.trajectoryBuilder(lastPose)
                            .lineToSplineHeading(new Pose2d(12, 0, Math.toRadians(0)))
                            .build();
                    lastPose = trajPark.end();
                } else {
                    trajSafePositionAfterWobbleTargetDrop = hzDrive.trajectoryBuilder(lastPose)
                            .lineToSplineHeading(new Pose2d(36, 36, Math.toRadians(0)))
                            .build();
                    lastPose = trajSafePositionAfterWobbleTargetDrop.end();
                    trajParkWithoutPickRing = hzDrive.trajectoryBuilder(lastPose)
                            .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(0)))
                            .build();
                    lastPose = trajParkWithoutPickRing.end();
                }
                break;

            case C:
                trajWobbleDropPosition = hzDrive.trajectoryBuilder(lastPose)
                        .lineToSplineHeading(new Pose2d(35,50,Math.toRadians(-135)))
                        .build();
                lastPose = trajWobbleDropPosition.end();
                if (hzAutonomousController.pickRingFromTargetMarker == true) {
                    trajPickRingsFromTargetMark = hzDrive.trajectoryBuilder(lastPose)
                            .lineToLinearHeading(new Pose2d(-5, 36, Math.toRadians(-180)))
                            .lineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(-180)))
                            .lineToLinearHeading(new Pose2d(-20, 36, Math.toRadians(-180)))
                            .build();
                    lastPose = trajPickRingsFromTargetMark.end();
                    if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal == true) {
                        trajTurnToHighGoal = hzDrive.trajectoryBuilder(lastPose)
                                .lineToLinearHeading(new Pose2d(-5, 36, Math.toRadians(0)))
                                .build();
                        lastPose = trajTurnToHighGoal.end();
                    }
                    trajSafePostionBeforePark =  hzDrive.trajectoryBuilder(lastPose)
                            .lineToLinearHeading(new Pose2d(-48,22,Math.toRadians(-90)))
                            .build();
                    lastPose = trajSafePostionBeforePark.end();
                    trajPark = hzDrive.trajectoryBuilder(lastPose)
                            .lineToSplineHeading(new Pose2d(12, 0, Math.toRadians(0)))
                            .build();
                    lastPose = trajPark.end();
                } else {
                    trajSafePositionAfterWobbleTargetDrop = hzDrive.trajectoryBuilder(lastPose)
                            .lineToSplineHeading(new Pose2d(36, 36, Math.toRadians(0)))
                            .build();
                    lastPose = trajSafePositionAfterWobbleTargetDrop.end();
                    trajParkWithoutPickRing = hzDrive.trajectoryBuilder(lastPose)
                            .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(0)))
                            .build();
                    lastPose = trajParkWithoutPickRing.end();
                }
                break;
        }
    }


    public void runAutoBlueOuter(){

        switch(currentAutoStepState) {
            case INITIATE:
                hzAutonomousController.setMagazineToLaunch();
                if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
                    hzAutonomousController.setLaunchTargetHighGoal();
                    currentAutoStepState = AutoStepState.TRAJ_LAUNCH_RINGS_HIGHGOAL;
                } else {
                    hzAutonomousController.setLaunchTargetPowerShot1();
                    currentAutoStepState = AutoStepState.TRAJ_LAUNCH_RING_POWERSHOT1;
                }
                break;

            case TRAJ_LAUNCH_RINGS_HIGHGOAL:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajLaunchRingsHighGoal);
                    currentAutoStepState = AutoStepState.LAUNCH_RINGS_HIGHGOAL;
                }
                break;

            case LAUNCH_RINGS_HIGHGOAL:
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();
                hzWait(500);
                hzAutonomousController.setRunLauncherTrue();
                hzWait(350);
                hzAutonomousController.setRunLauncherTrue();
                hzWait(350);
                hzAutonomousController.setRunLauncherTrue();
                hzWait(200);
                hzAutonomousController.setLaunchTargetOff();
                hzAutonomousController.setMagazineToCollect();
                currentAutoStepState = AutoStepState.TRAJ_WOBBLE_DROP_POSITION;
                break;

            case TRAJ_LAUNCH_RING_POWERSHOT1:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajLaunchRingPowershot1);
                    currentAutoStepState = AutoStepState.LAUNCH_RINGS_POWERSHOT1;
                }
                break;

            case LAUNCH_RINGS_POWERSHOT1:
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetPowerShot1();
                hzWait(500);
                hzAutonomousController.setRunLauncherTrue();
                currentAutoStepState = AutoStepState.TRAJ_LAUNCH_RING_POWERSHOT2;
                break;

            case TRAJ_LAUNCH_RING_POWERSHOT2:
                if (!hzDrive.isBusy()) {
                    hzDrive.turnAsync(tuneAnglePowershot12);
                    currentAutoStepState = AutoStepState.LAUNCH_RINGS_POWERSHOT2;
                }
                break;

            case LAUNCH_RINGS_POWERSHOT2:
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetPowerShot2();
                hzWait(500);
                hzAutonomousController.setRunLauncherTrue();
                currentAutoStepState = AutoStepState.TRAJ_LAUNCH_RING_POWERSHOT3;
                break;

            case TRAJ_LAUNCH_RING_POWERSHOT3:
                if (!hzDrive.isBusy()) {
                    hzDrive.turnAsync(tuneAnglePowershot23);
                    currentAutoStepState = AutoStepState.LAUNCH_RINGS_POWERSHOT3;
                }
                break;

            case LAUNCH_RINGS_POWERSHOT3:
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetPowerShot3();
                hzWait(500);
                hzAutonomousController.setRunLauncherTrue();
                currentAutoStepState = AutoStepState.TRAJ_WOBBLE_DROP_POSITION;
                break;

            case TRAJ_WOBBLE_DROP_POSITION:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajWobbleDropPosition);
                    currentAutoStepState = AutoStepState.DROP_WOBBLE_GOAL_ON_TARGET;
                }
                break;

            case DROP_WOBBLE_GOAL_ON_TARGET:
                hzAutonomousController.setMoveArmDropWobbleAutonoumous();
                hzWait(1000);
                hzAutonomousController.runOpenGrip();
                hzWait(500);
                hzAutonomousController.setMoveArmParked();
                if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A) {
                    currentAutoStepState = AutoStepState.TRAJ_SAFE_POSITION_AFTER_WOBBLE_TARGET_DROP;
                } else {
                    if (hzAutonomousController.pickRingFromTargetMarker == true) {
                        currentAutoStepState = AutoStepState.START_INTAKE;
                    } else {
                        currentAutoStepState = AutoStepState.TRAJ_SAFE_POSITION_AFTER_WOBBLE_TARGET_DROP;
                    }
                }
                break;

            case TRAJ_SAFE_POSITION_AFTER_WOBBLE_TARGET_DROP:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajSafePositionAfterWobbleTargetDrop);
                    currentAutoStepState = AutoStepState.TRAJ_PARK_WITHOUT_PICK_RING;
                }
                break;

            case TRAJ_PARK_WITHOUT_PICK_RING:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajParkWithoutPickRing);
                    currentAutoStepState = AutoStepState.END;
                }
                break;

            case START_INTAKE:
                hzIntakeUltimateGoal.setIntakeReleaseOpen();
                hzAutonomousController.setIntakeStart();
                currentAutoStepState = AutoStepState.TRAJ_PICK_RINGS_FROM_TARGET_MARK;
                break;

            case TRAJ_PICK_RINGS_FROM_TARGET_MARK:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajPickRingsFromTargetMark);
                    if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal == true){
                        currentAutoStepState = AutoStepState.TRAJ_TURN_TO_HIGH_GOAL;
                    } else {
                        currentAutoStepState = AutoStepState.TRAJ_SAFE_POSITION_BEFORE_PARK;
                    }
                }
                break;

            case TRAJ_TURN_TO_HIGH_GOAL:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajTurnToHighGoal);
                    currentAutoStepState = AutoStepState.LAUNCH_RINGS_HIGHGOAL_SECOND;
                }
                break;

            case LAUNCH_RINGS_HIGHGOAL_SECOND:
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();
                hzWait(500);
                hzAutonomousController.setRunLauncherTrue();
                hzWait(350);
                hzAutonomousController.setRunLauncherTrue();
                hzWait(350);
                hzAutonomousController.setRunLauncherTrue();
                hzWait(200);
                hzAutonomousController.setLaunchTargetOff();
                hzAutonomousController.setMagazineToCollect();
                currentAutoStepState = AutoStepState.TRAJ_SAFE_POSITION_BEFORE_PARK;
                break;

            case TRAJ_SAFE_POSITION_BEFORE_PARK:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajSafePostionBeforePark);
                    currentAutoStepState = AutoStepState.TRAJ_PARK;
                }
                break;

            case TRAJ_PARK:
                if (!hzDrive.isBusy()) {
                    hzDrive.followTrajectoryAsync(trajPark);
                    currentAutoStepState = AutoStepState.END;
                }
                break;

            case END:
                hzAutonomousController.setIntakeStop();
                hzIntakeUltimateGoal.setIntakeReleaseOpen();
                currentAutoStepState = AutoStepState.IDLE;
                break;

            case IDLE:
                parked = true;
                break;
        }
    }


    public void hzWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){}
    }

    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "4:11 :: 1/14");

        //***** Select Alliance ******

        HzGameFieldUltimateGoal.playingAlliance = HzGameFieldUltimateGoal.PLAYING_ALLIANCE.BLUE_ALLIANCE;
        startPose = HzGameFieldUltimateGoal.BLUE_OUTER_START_LINE;
        activeWebcam = HzVuforiaUltimateGoal.ACTIVE_WEBCAM.LEFT;

        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameFieldUltimateGoal.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ",activeWebcam);
            telemetry.addData("Please select launch target : ", "(B) for High Goal, (X) for Powershot");
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
            //telemetry.addData("10s time : Default Alliance A :",);
            telemetry.update();
            hzWait(200);
        }

        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameFieldUltimateGoal.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ",activeWebcam);
            telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutonomousController.autoLaunchAim);
            telemetry.addData("Please select option for Picking rings from target markers and launching : ", "");
            telemetry.addData("    (A):","Park after Wobble Goal (No ring Pick)");
            telemetry.addData("    (Y):","Pick rings after Wobble Goal and park (No launching)");
            telemetry.addData("    (X):","Pick rings and launch after Wobble Goal, and park");
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


        telemetry.addData("Playing Alliance :", HzGameFieldUltimateGoal.playingAlliance);
        telemetry.addData("startPose : ", startPose);
        telemetry.addData("activeWebcam : ",activeWebcam);
        telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutonomousController.autoLaunchAim);
        telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutonomousController.pickRingFromTargetMarker);
        telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal);
        telemetry.addData("autonomousStarted : ", autonomousStarted);
        telemetry.addData("currentAutoStepState : ", currentAutoStepState);
        telemetry.addData("lastPose : ", lastPose);

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
