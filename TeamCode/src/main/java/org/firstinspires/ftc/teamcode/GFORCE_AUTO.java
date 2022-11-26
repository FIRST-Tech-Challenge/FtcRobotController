package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@Autonomous(name="G-FORCE AUTO", group="!Competition", preselectTeleOp="G-FORCE TELEOP")
public class GFORCE_AUTO extends LinearOpMode {

    private Elevator elevator;
    private ConeTracker coneTracker;
    private GFORCE_Vision vision;
    AutoConfig autoConfig = new AutoConfig();
    boolean weAreRed;
    GFORCE_KiwiDrive drive = null;

    boolean trackConeNow = false;
    boolean grabConeWhenReady = false;
    boolean dropConeWhenReady = false;
    boolean coneGrabbed = false;

    boolean lookForSignalImage = false;
    int     foundSignalImage = 0;

    final Double TILEx1_0 = 23.5;
    final Double TILEx0_5 = TILEx1_0 * 0.5;
    final Double TILEx1_5 = TILEx1_0 * 1.5;
    final Double TILEx2_0 = TILEx1_0 * 2.0;
    final Double TILEx2_5 = TILEx1_0 * 2.5;

    // declare all trajectories //
    TrajectorySequence redFrontJunctionInit;
    TrajectorySequence redFrontJunctionTransition;
    TrajectorySequence redFrontJunctionLoop;

    Pose2d redFrontStartPosition = new Pose2d(-62, TILEx1_5,  Math.toRadians(0));  // Auto Start
    Pose2d redBackStartPosition  = new Pose2d(-62, -TILEx1_5, Math.toRadians(0));  // Auto Start

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize GFORCE_KiwiDrive
        drive = new GFORCE_KiwiDrive(hardwareMap);
        autoConfig.init(this);
        elevator = new Elevator(this, true);
        coneTracker = new ConeTracker(this);
        vision = new GFORCE_Vision(this);
        vision.init();

        // Select the initial trajectory
        setupTrajectories();

        while (opModeInInit()) {
            elevator.update();
            elevator.runStateMachine();
            //read signal cone
            vision.getSignalNumber();

            // Select new auto options and trajectories
            if (autoConfig.init_loop()) {
                setupTrajectories();
            }
        }

        if (opModeIsActive()) {
            if (autoConfig.autoOptions.enabled) {
                weAreRed = autoConfig.autoOptions.redAlliance;
                if (weAreRed) {
                    if (autoConfig.autoOptions.startFront) {

                        if (autoConfig.autoOptions.scoreJunction) {
                            //we are red, starting at the front, scoring the junction
                            buildRedFrontJunction();
                            followGforceSequence(redFrontJunctionInit);
//                            followGforceSequence(redFrontJunctionTransition);
                            followGforceSequence(redFrontJunctionLoop);
                            followGforceSequence(redFrontJunctionLoop);
                        }
                    }
                }
            }
        }

        // save the last field position in shared states class for teleop to use.
        SharedStates.currentPose = drive.getPoseEstimate();

        // Set Elevator state back to Idle for next autonomous
        elevator.setState(ElevatorState.IDLE);
    }

    // Setup the trajectories for the current selected Autonomous options.
    private void setupTrajectories() {
        weAreRed = autoConfig.autoOptions.redAlliance;
        if ((weAreRed && autoConfig.autoOptions.startFront) ||
                (!weAreRed && !autoConfig.autoOptions.startFront)) {
            buildRedFrontJunction();
        } else {
            buildRedRearJunction();
        }
    }

    // follow the current trajectory, while checking for active cone tracking.
    public void followGforceSequence(TrajectorySequence trajectory) {
        int  tempImage;
        drive.followTrajectorySequenceAsync(trajectory);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            elevator.update();
            elevator.runStateMachine();
            //  elevator.showElevatorState();
            telemetry.update();

            if (!trackConeNow) {
                drive.update();
            } else {
                coneAutoHoming();
            }

            if (lookForSignalImage) {
                if ((tempImage = vision.getSignalNumber()) > 0) {
                    foundSignalImage = tempImage;
                }
            }

            if (foundSignalImage > 0) {
                telemetry.addData("Signal Number", foundSignalImage);
                telemetry.update();
            }
        }
    }

    // use in TemporalMarker start tracking the cone.
    public void startConeTracking() {
        coneTracker.reset();
        coneGrabbed = false;
        trackConeNow = true;
    }

    // Home in on the nearest cone.
    public void coneAutoHoming() {
        drive.updatePoseEstimate();
        if (coneTracker.update() && !coneGrabbed) {
            // coneTracker.showRanges();
            drive.setWeightedDrivePower(new Pose2d(coneTracker.trackDrive(), 0, coneTracker.trackTurn()));

            if (grabConeWhenReady && coneTracker.trackGrab()) {
                coneGrabbed = true;
                trackConeNow = false;
                grabConeWhenReady = false;
                elevator.autoGrab();
            }

            if (dropConeWhenReady && coneTracker.trackGrab()) {
                trackConeNow = false;
                dropConeWhenReady = false;
                elevator.autoRelease();
            }

        } else {
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        }
        telemetry.update();
    }

    // trajectory builders //
    private void buildRedFrontJunction() {

        Pose2d redFrontStartPosition = new Pose2d(-62, 35, Math.toRadians(0));        // Auto Start

        drive.setExternalHeading(Math.toRadians(0));
        drive.setPoseEstimate(redFrontStartPosition);

        //=========================================================================
        redFrontJunctionInit = drive.trajectorySequenceBuilder(redFrontStartPosition)
            // Move forward and raise lift.  Start looking for signal image
            .addDisplacementMarker(0.5, () -> {
                elevator.setLiftTargetPosition(Elevator.ELEVATOR_LOW);
                lookForSignalImage = true;
            })
            .lineTo(new Vector2d(-TILEx2_0 - 10, TILEx1_5))
            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                lookForSignalImage = false;
            })

            //  Turn sideways and translate to front of junction.  Release cone once there.
            .turn(Math.toRadians(90))
            .lineTo(new Vector2d(-TILEx1_0, TILEx1_5 + 2.5))
            .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                elevator.autoRelease();
            })
            .waitSeconds(1.0)

            // Translate again to be in center of tile parking)
            .strafeRight(2)
            .splineToConstantHeading(new Vector2d(-TILEx0_5, TILEx2_0), Math.toRadians(90))
            .lineTo(new Vector2d(-TILEx0_5, TILEx2_0 + 8.5)) // center of scoring...
            .build();

        //=========================================================================================
       // redFrontJunctionTransition = drive.trajectorySequenceBuilder(redFrontJunctionInit.end())

        //        .lineTo(new Vector2d(-TILEx0_5, TILEx2_0 + 8.5)) // center of scoring...

        //        .build();

        //=========================================================================================
        redFrontJunctionLoop = drive.trajectorySequenceBuilder(redFrontJunctionInit.end())

            // Track cone and Grab cone when in position.
            .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                elevator.setWristOffset(0);
                elevator.setHandPosition(elevator.HAND_OPEN);
            })
            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                grabConeWhenReady = true;
                startConeTracking();
            })
            .waitSeconds(1.5)

            // Back away from conestack and turn to junction.
            .setReversed(true)
            .lineTo(new Vector2d(-TILEx0_5 , TILEx2_0 + 10.5))
            .lineTo(new Vector2d(-TILEx0_5, TILEx2_0 + 8.5))
            .turn(Math.toRadians(135))
            .setReversed(false)

            // Track to cone, and Release cone when in position
            .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                dropConeWhenReady = true;
                startConeTracking();
                elevator.dropStackHeight();
            })
            .waitSeconds(1.5)

            // Back away from junction and turn to cone-stack.
            .setReversed(true)
            .lineTo(new Vector2d(-TILEx0_5 - 2, TILEx2_0 + 6.5))
            .lineTo(new Vector2d(-TILEx0_5, TILEx2_0 + 8.5))
            .turn(Math.toRadians(-135))
            .setReversed(false)
            .build();
    }

    //=========================================================================
    private void buildRedRearJunction() {

    }
}