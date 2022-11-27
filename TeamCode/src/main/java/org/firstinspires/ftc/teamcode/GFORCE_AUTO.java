package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_PUSH_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.GFORCE_KiwiDrive.getVelocityConstraint;

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
    TrajectorySequence redFrontReadSignal;
    TrajectorySequence redFrontScoreJunction;
    TrajectorySequence redFrontJunctionTransition;
    TrajectorySequence redFrontJunctionLoop1;
    TrajectorySequence redFrontJunctionLoop2;
    TrajectorySequence redFrontJunctionLoop3;
    TrajectorySequence redFrontSignalPark1;
    TrajectorySequence redFrontSignalPark2;
    TrajectorySequence redFrontSignalPark3;
    TrajectorySequence redFrontJunctionPark1;
    TrajectorySequence redFrontJunctionPark2;
    TrajectorySequence redFrontJunctionPark3;
    TrajectorySequence redFrontStackPark1;
    TrajectorySequence redFrontStackPark2;
    TrajectorySequence redFrontStackPark3;

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

        elevator.setWristOffset(80);

        if (opModeIsActive()) {
            if (autoConfig.autoOptions.enabled) {
                weAreRed = autoConfig.autoOptions.redAlliance;
                if (weAreRed) {
                    if (autoConfig.autoOptions.startFront) {
                        if (autoConfig.autoOptions.scoreJunction) {
                            // we are red, starting at the front, scoring the junction
                            followGforceSequence(redFrontReadSignal);
                            followGforceSequence(redFrontScoreJunction);
                            followGforceSequence(redFrontJunctionTransition);

                            if (autoConfig.autoOptions.scoreConeStack) {
                                // we are red, starting at the front, scoring the junction, and the cone stack
                                followGforceSequence(redFrontJunctionLoop1);
                                followGforceSequence(redFrontJunctionLoop2);
                                followGforceSequence(redFrontJunctionLoop3);

                                followGforceSequence(redFrontJunctionLoop1);
                                followGforceSequence(redFrontJunctionLoop2);

                                if (autoConfig.autoOptions.park) {
                                    // we are red, starting at the front, scoring the junction, and the cone stack, and parking
                                    // we are red, starting at the front, scoring the junction and Not Conestack, and parking
                                    if (foundSignalImage == 3)
                                        followGforceSequence(redFrontStackPark3);
                                    else if (foundSignalImage == 2)
                                        followGforceSequence(redFrontStackPark2);
                                    else
                                        followGforceSequence(redFrontStackPark1);
                                } else {
                                    // we are red, starting at the front, scoring the junction and Conestack, but NOT parking
                                    followGforceSequence(redFrontJunctionLoop3);
                                    followGforceSequence(redFrontJunctionLoop1);
                                    followGforceSequence(redFrontJunctionLoop2);
                                }
                            } else {
                                // we are red, starting at the front, scoring the junction and Not Conestack
                                if (autoConfig.autoOptions.park) {
                                    // we are red, starting at the front, scoring the junction and Not Conestack, and parking
                                    if (foundSignalImage == 3)
                                        followGforceSequence(redFrontJunctionPark3);
                                    else if (foundSignalImage == 2)
                                        followGforceSequence(redFrontJunctionPark2);
                                    else
                                        followGforceSequence(redFrontJunctionPark1);
                                }
                            }
                        } else {
                            // we are red, starting at the front, NOT scoring the junction
                            if (autoConfig.autoOptions.park) {
                                // we are red, starting at the front, only parking
                                followGforceSequence(redFrontReadSignal);

                                // we are red, starting at the front, NOT scoring the junction and parking
                                if (foundSignalImage == 3)
                                    followGforceSequence(redFrontSignalPark3);
                                else if (foundSignalImage == 2)
                                    followGforceSequence(redFrontSignalPark2);
                                else
                                    followGforceSequence(redFrontSignalPark1);
                            }
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
        while (!Thread.currentThread().isInterrupted() && drive.isBusy() && !elevator.sequenceComplete()) {
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
        elevator.sequenceComplete();
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

        Pose2d redFrontStartPosition = new Pose2d(-62, TILEx1_5,  Math.toRadians(0));  // Auto Start

        drive.setExternalHeading(Math.toRadians(0));
        drive.setPoseEstimate(redFrontStartPosition);

        //=========================================================================================
        // JUST read the signal
        redFrontReadSignal = drive.trajectorySequenceBuilder(redFrontStartPosition)
            .waitSeconds(0.1)
            // Move forward and raise lift.  Start looking for signal image
            .addDisplacementMarker(0.1, () -> {
                elevator.setLiftTargetPosition(Elevator.ELEVATOR_LOW);
                lookForSignalImage = true;
            })
            .lineTo(new Vector2d(-TILEx2_0 - 10, TILEx1_5))
            .build();

        //=========================================================================
        redFrontScoreJunction = drive.trajectorySequenceBuilder(redFrontReadSignal.end())
            //  Turn sideways and translate to front of junction.  Release cone once there.
            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                lookForSignalImage = false;
                elevator.setWristOffset(0);
            })
            .turn(Math.toRadians(90))
            .lineTo(new Vector2d(-TILEx1_0, TILEx1_5 + 2.5))
            .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { elevator.autoRelease();  })
            .waitSeconds(3.0)
            .build();

        // Translate again to be in center of tile parking)
        redFrontJunctionTransition = drive.trajectorySequenceBuilder(redFrontScoreJunction.end())
            .strafeRight(2)
            .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                elevator.setWristOffset(0);
                elevator.setHandPosition(elevator.HAND_OPEN);
            })
            .splineToConstantHeading(new Vector2d(-TILEx0_5, TILEx2_0), Math.toRadians(90))
            .lineTo(new Vector2d(-TILEx0_5, TILEx2_0 + 8)) // center of scoring...
            .build();

        //=========================================================================================
        // Track to, and pickup cone from stack
        redFrontJunctionLoop1 = drive.trajectorySequenceBuilder(redFrontJunctionTransition.end())

            // Track cone and Grab cone when in position.
            .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                grabConeWhenReady = true;
                startConeTracking();
            })
            .waitSeconds(3)
            .build();

        //=========================================================================================
        // Back away from stack, turn to junction.  Track cone and drop Cone.
        redFrontJunctionLoop2 = drive.trajectorySequenceBuilder(new Pose2d(-TILEx0_5 , TILEx2_0 + 10, Math.toRadians(90)))
            // Back away from conestack and turn to junction.
            .setReversed(true)
            .lineTo(new Vector2d(-TILEx0_5, TILEx2_0 + 8))
            .turn(Math.toRadians(135))
            .setReversed(false)

            // Track to cone, and Release cone when in position
            .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                dropConeWhenReady = true;
                startConeTracking();
                elevator.dropStackHeight();
            })
            .waitSeconds(3)
            .build();

        //=========================================================================================
        // Back away from Junction, turn to stack.
        redFrontJunctionLoop3 = drive.trajectorySequenceBuilder(new Pose2d(-TILEx0_5 - 2, TILEx2_0 + 6, Math.toRadians(-135)))
            // Back away from junction and turn to cone-stack.
            .setReversed(true)
            .lineTo(new Vector2d(-TILEx0_5, TILEx2_0 + 8))
            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                elevator.setWristOffset(0);
                elevator.setHandPosition(elevator.HAND_OPEN);
            })
            .turn(Math.toRadians(-135))
            .setReversed(false)
            .build();

        //=========================================================================================
        redFrontStackPark1 = makeRedStackPark(1);
        redFrontStackPark2 = makeRedStackPark(2);
        redFrontStackPark3 = makeRedStackPark(3);

        redFrontJunctionPark1 = makeRedJunctionPark(1);
        redFrontJunctionPark2 = makeRedJunctionPark(2);
        redFrontJunctionPark3 = makeRedJunctionPark(3);

        // Drive straight to Park at Location 1
        redFrontSignalPark1 = makeRedSignalPark(1);
        redFrontSignalPark2 = makeRedSignalPark(2);
        redFrontSignalPark3 = makeRedSignalPark(3);
    }

    TrajectorySequence makeRedSignalPark(int signal) {
        TrajectorySequence tempSeq = drive.trajectorySequenceBuilder(redFrontReadSignal.end())
                // Make a run from the signal detection point
                .setVelConstraint(getVelocityConstraint(MAX_PUSH_VEL, MAX_ANG_VEL, TRACK_WIDTH))
                .lineTo(new Vector2d(-TILEx0_5 + 5, TILEx1_5))
                .back(6)
                .lineTo(new Vector2d(-TILEx0_5 + -1, (TILEx1_5 - 1) - ((signal - 2) * TILEx1_0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { elevator.setWristOffset(0); })
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> { elevator.setLiftTargetPosition(Elevator.ELEVATOR_HOME); })
                .waitSeconds(4)
                .build();

        return tempSeq;
    }

    TrajectorySequence makeRedJunctionPark(int signal) {
        TrajectorySequence tempSeq = drive.trajectorySequenceBuilder(redFrontJunctionTransition.end())
                // Back away from junction and turn to cone-stack.
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> { elevator.setLiftTargetPosition(Elevator.ELEVATOR_LOW); })
                .waitSeconds(1)
                .lineTo(new Vector2d(-TILEx0_5, (TILEx1_5 - 2) - ((signal - 2) * TILEx1_0)))
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    elevator.setLiftTargetPosition(Elevator.ELEVATOR_HOME);
                })
                .waitSeconds(2)
                .build();

        return tempSeq;
    }

    TrajectorySequence makeRedStackPark(int signal) {
        TrajectorySequence tempSeq = drive.trajectorySequenceBuilder(new Pose2d(-TILEx0_5 - 2, TILEx2_0 + 6, Math.toRadians(-135)))
                // Back away from junction and turn to cone-stack.
                .setReversed(true)
                .lineTo(new Vector2d(-TILEx0_5, TILEx1_5 - ((signal - 2) * TILEx1_0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { elevator.setHandPosition(elevator.HAND_OPEN); })
                .turn(Math.toRadians(-135))
                .setReversed(false)
                .build();

        return tempSeq;
    }


    //=========================================================================
    private void buildRedRearJunction() {
        Pose2d redRearStartPosition  = new Pose2d(-62, -TILEx1_5, Math.toRadians(0));  // Auto Start

    }
}