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
@Autonomous(name = "Hazmat Autonomous 2", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp RR")
@Disabled
public class HzAutonomousBasic2 extends LinearOpMode {

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
        af = HzGameFieldUltimateGoal.ALLIANCE_FACTOR;

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
            hzAutonomousController.setMagazineToLaunch();
            hzAutonomousController.runAutoControl();

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
            hzLaunchSubControllerUltimateGoal.launchMode = HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.MANUAL;

            while (opModeIsActive() && !isStopRequested() && !parked) {

                hzVuforiaUltimateGoal.deactivateVuforiaTensorFlow();

                if (HzGameFieldUltimateGoal.startPosition == HzGameFieldUltimateGoal.START_POSITION.INNER) {
                    runAutoInner();
                } else { //HzGameField.startPosition == HzGameField.START_POSITION.OUTER
                    runAutoOuter();
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

    public void runAutoInner(){

        // Set magazine to Launch in case it slipped
        hzAutonomousController.setMagazineToLaunch();

        // Move to launch position and launch rings to High Goal or Powershots
        if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
            hzAutonomousController.setLaunchTargetHighGoal();
            if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-40, af * 6, Math.toRadians(af * 45)))
                        .build();
                hzDrive.followTrajectory(traj);
            }
            hzAutonomousController.setMagazineToLaunch();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,af*14,Math.toRadians(af*18)))
                    .build();
            hzDrive.followTrajectory(traj);

            launch3RingsToHighGoal();
        } else {
            hzAutonomousController.setLaunchTargetPowerShot1();
            if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-40, af * 6, Math.toRadians(af * 45)))
                        .build();
                hzDrive.followTrajectory(traj);
            }
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,af*14,Math.toRadians(af*5)))
                    .build();
            hzDrive.followTrajectory(traj);

            //Set turn angles prior to launching
            turnAnglePowershot12 = Math.toRadians(af*-5);
            turnAnglePowershot23 = Math.toRadians(af*-5);
            launch3RingsToPowerShots();
        }

        // Move to drop wobble goal on target
        switch (targetZone){
            case A:
                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(46, af * 22, Math.toRadians(af * -45)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(27,af*43,Math.toRadians(af*-45)))
                        .build();
                hzDrive.followTrajectory(traj);

                break;

            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(46,af*22,Math.toRadians(af*-45)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(54,af*51,Math.toRadians(af*-90)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;

        }

        dropWobbleGoalInTarget();

        if ((hzAutonomousController.pickRingFromTargetMarker == false) || (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A)){ // Park
            //Park
            if (hzAutonomousController.pickAndDropSecondWobbleGoal) {
                runInnerPickAndDropSecondWobbleGoalAndPark();
            } else {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(50, af * 16, Math.toRadians(af * -45)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af * 20, Math.toRadians(af * 0)))
                        .build();
                hzDrive.followTrajectory(traj);
            }

            /* Alternate
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(10, af*0, Math.toRadians(0)))
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
                        .lineToLinearHeading(new Pose2d(-10,af*37,Math.toRadians(af*135)))
                        .build();
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22,af*37,Math.toRadians(af*135)))
                        .build();
                hzDrive.followTrajectory(traj);

                hzWait(300);
            }

            // Spline to (24,24,0)
            if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10,af*37,Math.toRadians(af*-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-22, af * 37, Math.toRadians(af * -180)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-33, af*37, Math.toRadians(af*-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                hzWait(300);
            }

            if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal) {
                hzAutonomousController.setIntakeStop();
                hzAutonomousController.setMagazineToLaunch();
                hzAutonomousController.setLaunchTargetHighGoal();

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-13, af*40, Math.toRadians(af*0)))
                        .build();
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();

                if(hzAutonomousController.pickAndDropSecondWobbleGoal == true) {
                    runInnerPickAndDropSecondWobbleGoalAndPark();
                } else{
                    //Park
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(13, af*33, Math.toRadians(af*0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }


            } else { //Move to baseline and park in safe zone
                traj =  hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-50,af*16,Math.toRadians(af*-90)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(10, af*0, Math.toRadians(af*0)))
                        .build();
                hzDrive.followTrajectory(traj);
                hzAutonomousController.setIntakeStop();
            }
        }
    }

    public void runInnerPickAndDropSecondWobbleGoalAndPark() {
        hzAutonomousController.setMoveArmPickWobble();
        hzAutonomousController.runOpenGrip();
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-33, af*50, Math.toRadians(af*0)))
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
        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
        }
    }

    public void runAutoOuter(){
        // Set magazine to Launch in case it slipped
        hzAutonomousController.setMagazineToLaunch();

        // Move to launch position and launch rings to High Goal or Powershots
        if (hzAutonomousController.autoLaunchAim == HzAutonomousControllerUltimateGoal.AutoLaunchAim.HIGHGOAL) {
            hzAutonomousController.setLaunchTargetHighGoal();
            if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-40, af * 50, Math.toRadians(af * -45)))
                        .build();
                hzDrive.followTrajectory(traj);
            }
            hzAutonomousController.setMagazineToLaunch();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,af*50,Math.toRadians(af*-10)))
                    .build();
            hzDrive.followTrajectory(traj);

            launch3RingsToHighGoal();
        } else {
            hzAutonomousController.setLaunchTargetPowerShot1();
            if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-40, af * 50, Math.toRadians(af * 45)))
                        .build();
                hzDrive.followTrajectory(traj);
            }
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-10,af*50,Math.toRadians(af*-15)),Math.toRadians(0))
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
                        .lineToLinearHeading(new Pose2d(-10,af*48,Math.toRadians(af*-135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(15,af*48,Math.toRadians(af*135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(35,af*50,Math.toRadians(af*-150)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;

        }

        dropWobbleGoalInTarget();

        if ((hzAutonomousController.pickRingFromTargetMarker == false) || (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.A)){ // Park
            if (hzAutonomousController.pickAndDropSecondWobbleGoal) {
                runOuterPickAndDropSecondWobbleGoalAndPark();
            } else {
            //Park
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af * 20, Math.toRadians(af * 0)))
                        .build();
                hzDrive.followTrajectory(traj);

            /* Alternate
            traj =  hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-50,af*16,Math.toRadians(af*-90)))
                    .build();
            hzDrive.followTrajectory(traj);
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(10, af*0, Math.toRadians(af*0)))
                    .build();
            hzDrive.followTrajectory(traj);
            */
            }
            return;

        } else { //Target Zone is B or C and pickRingFromTargetMarker == True
            //Pick rings from Target Marker
            hzIntakeUltimateGoal.setIntakeReleaseOpen();
            hzAutonomousController.setIntakeStart();
            hzWait(500);

            if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.B) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10,af*33,Math.toRadians(af*135)))
                        .build();
                hzDrive.followTrajectory(traj);

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-22,af*33,Math.toRadians(af*135)))
                        .build();
                hzDrive.followTrajectory(traj);

                hzWait(300);
            }

            // Spline to (24,24,0)
            if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-10,af*33,Math.toRadians(af*-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                if (!hzAutonomousController.pickAndDropSecondWobbleGoal) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-22, af * 33, Math.toRadians(af * -180)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-33, af*33, Math.toRadians(af*-180)))
                        .build();
                hzDrive.followTrajectory(traj);

                hzWait(300);
            }

            if (hzAutonomousController.launchRingsPickedFromTargetMarkerToHighGoal) {
                hzAutonomousController.setIntakeStop();
                hzAutonomousController.setLaunchTargetHighGoal();
                hzAutonomousController.setMagazineToLaunch();

                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-13, af*40, Math.toRadians(af*0)))
                        .build();
                hzDrive.followTrajectory(traj);

                launch3RingsToHighGoal();

                if(hzAutonomousController.pickAndDropSecondWobbleGoal == true) {
                    runOuterPickAndDropSecondWobbleGoalAndPark();
                } else {

                    //Park
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }
            } else { //Move to baseline and park in safe zone
                traj =  hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-50,af*16,Math.toRadians(af*-90)))
                        .build();
                hzDrive.followTrajectory(traj);
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(10, af*0, Math.toRadians(af*0)))
                        .build();
                hzDrive.followTrajectory(traj);
                hzAutonomousController.setIntakeStop();
            }
        }
    }

    public void runOuterPickAndDropSecondWobbleGoalAndPark() {
        hzAutonomousController.setMoveArmPickWobble();
        hzAutonomousController.runOpenGrip();
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-31, af*23, Math.toRadians(af*0)))
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
        if (targetZone == HzGameFieldUltimateGoal.TARGET_ZONE.C) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
        }
    }

    public void launch3RingsToHighGoal(){
        hzAutonomousController.setLaunchTargetHighGoal();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(200);
        hzAutonomousController.setLaunchTargetHighGoal();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(500);
        hzAutonomousController.setRunLauncherTrue();
        hzAutonomousController.setMagazineToLaunch();
        hzWait(400);
        hzAutonomousController.setRunLauncherTrue();
        hzWait(600);

        hzAutonomousController.setLaunchTargetOff();
        hzAutonomousController.setMagazineToCollect();
    }

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

    public void dropWobbleGoalInTarget(){
        hzAutonomousController.setMoveArmDropWobbleAutonoumous();
        //hzWait(1000);
        hzWait(500);
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

    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "4:11 :: 1/14");

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
            telemetry.addData("    (Y):","Launch and park");
            telemetry.addData("    (A):","Launch, drop WG and park");
            telemetry.addData("    (X):","Launch, drop WG, Pick rings, launch and park");
            telemetry.addData("    (B):","Launch, drop WG, Pick rings, launch, move WG2 and park");

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
        telemetry.addData("HzGameField.ALLIANCE_FACTOR", HzGameFieldUltimateGoal.ALLIANCE_FACTOR);

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

