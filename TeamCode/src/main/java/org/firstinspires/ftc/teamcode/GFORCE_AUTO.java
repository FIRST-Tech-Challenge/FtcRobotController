package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_PUSH_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.GFORCE_KiwiDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    boolean coneStackOnLeft;

    GFORCE_KiwiDrive drive = null;

    boolean trackConeNow = false;
    boolean grabConeWhenReady = false;
    boolean dropConeWhenReady = false;
    boolean coneGrabbed = false;

    boolean lookForSignalImage = false;
    int     foundSignalImage = 0;

    ElapsedTime coneTrackTimer = new ElapsedTime();
    ElapsedTime autonomousTimer = new ElapsedTime();

    final Double TILEx1_0 = 23.5;

    final Double TILEx0_5 = TILEx1_0 * 0.5;
    final Double TILEx1_5 = TILEx1_0 * 1.5;
    final Double TILEx2_0 = TILEx1_0 * 2.0;
    final Double TILEx2_5 = TILEx1_0 * 2.5;
    final Double TILEx3_0 = TILEx1_0 * 3.0;

    // declare all trajectories //
    TrajectorySequence trjReadSignal;
    TrajectorySequence trjScoreJunction;
    TrajectorySequence trjJunctionTransition;
    TrajectorySequence trjJunctionLoop1;
    TrajectorySequence trjJunctionLoop2;
    TrajectorySequence trjJunctionLoop3;
    TrajectorySequence trjSignalPark1;
    TrajectorySequence trjSignalPark2;
    TrajectorySequence trjSignalPark3;
    TrajectorySequence trjJunctionPark1;
    TrajectorySequence trjJunctionPark2;
    TrajectorySequence trjJunctionPark3;
    TrajectorySequence trjStackPark1;
    TrajectorySequence trjStackPark2;
    TrajectorySequence trjStackPark3;

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
        weAreRed = autoConfig.autoOptions.redAlliance;
        setupTrajectories();

        while (opModeInInit()) {
            elevator.update();
            elevator.runStateMachine();
            vision.getSignalNumber();

            // Select new auto options and trajectories
            if (autoConfig.init_loop()) {
                weAreRed = autoConfig.autoOptions.redAlliance;
                setupTrajectories();
            }
        }

        // Start the timer and lift the wrist.
        autonomousTimer.reset();
        elevator.setWristOffset(80);

        if (opModeIsActive()) {
            if (autoConfig.autoOptions.enabled) {
                if (autoConfig.autoOptions.startFront) {
                    if (autoConfig.autoOptions.scoreJunction) {
                        // we are red, starting at the front, scoring the junction
                        followGforceSequence(trjReadSignal);
                        followGforceSequence(trjScoreJunction);
                        followGforceSequence(trjJunctionTransition);

                        if (autoConfig.autoOptions.scoreConeStack) {
                            // we are red, starting at the front, scoring the junction, and the cone stack
                            followGforceSequence(trjJunctionLoop1);
                            followGforceSequence(trjJunctionLoop2);

                            // Score an extra cone if we have time
                            if (autonomousTimer.time() < 19) {
                                followGforceSequence(trjJunctionLoop3);

                                if (autonomousTimer.time() < 21) {
                                    followGforceSequence(trjJunctionLoop1);

                                    if (autonomousTimer.time() < 24)
                                        followGforceSequence(trjJunctionLoop2);
                                }
                            }

                            if (autoConfig.autoOptions.park) {
                                // we are red, starting at the front, scoring the junction, and the cone stack, and parking
                                // we are red, starting at the front, scoring the junction and Not Conestack, and parking
                                if (foundSignalImage == 3)
                                    followGforceSequence(trjStackPark3);
                                else if (foundSignalImage == 2)
                                    followGforceSequence(trjStackPark2);
                                else
                                    followGforceSequence(trjStackPark1);
                            } else {
                                // we are red, starting at the front, scoring the junction and Conestack, but NOT parking
                                followGforceSequence(trjJunctionLoop3);
                                followGforceSequence(trjJunctionLoop1);
                                followGforceSequence(trjJunctionLoop2);
                            }
                        } else {
                            // we are red, starting at the front, scoring the junction and Not Conestack
                            if (autoConfig.autoOptions.park) {
                                // we are red, starting at the front, scoring the junction and Not Conestack, and parking
                                if (foundSignalImage == 3)
                                    followGforceSequence(trjJunctionPark3);
                                else if (foundSignalImage == 2)
                                    followGforceSequence(trjJunctionPark2);
                                else
                                    followGforceSequence(trjJunctionPark1);
                            }
                        }
                    } else {
                        // we are red, starting at the front, NOT scoring the junction
                        if (autoConfig.autoOptions.park) {
                            // we are red, starting at the front, only parking
                            followGforceSequence(trjReadSignal);

                            // we are red, starting at the front, NOT scoring the junction and parking
                            if (foundSignalImage == 3)
                                followGforceSequence(trjSignalPark3);
                            else if (foundSignalImage == 2)
                                followGforceSequence(trjSignalPark2);
                            else
                                followGforceSequence(trjSignalPark1);
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
        coneStackOnLeft = ((weAreRed && autoConfig.autoOptions.startFront) || (!weAreRed && !autoConfig.autoOptions.startFront));
        buildAutoTrajectories();
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

            if (trackConeNow && coneTrackTimer.time() < 3.0) {
                coneAutoHoming();
            } else {
                drive.update();
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
        coneTrackTimer.reset();
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

    // ================================================================================================================
    //  Trajectory Builders...  RED FRONT / BLUE REAR
    // ================================================================================================================
    private void buildAutoTrajectories() {

        Pose2d coneStackOnLeft = new Pose2d(-62, sideFlip(TILEx1_5),  Math.toRadians(0));  // Auto Start
        drive.setExternalHeading(Math.toRadians(0));
        drive.setPoseEstimate(coneStackOnLeft);

        //=========================================================================================
        // JUST read the signal
        trjReadSignal = drive.trajectorySequenceBuilder(coneStackOnLeft)
            .waitSeconds(0.1)
            // Move forward and raise lift.  Start looking for signal image
            .addDisplacementMarker(0.1, () -> {
                elevator.setLiftTargetPosition(Elevator.ELEVATOR_LOW);
                lookForSignalImage = true;
            })
            .lineTo(new Vector2d(-TILEx2_0 - 10, sideFlip(TILEx1_5)))
            .build();

        //=========================================================================
        trjScoreJunction = drive.trajectorySequenceBuilder(trjReadSignal.end())
            //  Turn sideways and translate to front of junction.  Release cone once there.
            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                lookForSignalImage = false;
                elevator.setWristOffset(0);
            })
            .turn(Math.toRadians(sideFlip(90)))
            .lineTo(new Vector2d(-TILEx1_0, sideFlip(TILEx1_5 + 2.5)))
            .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { elevator.autoRelease();  })
            .waitSeconds(3.0)
            .build();

        //=========================================================================================
        // Translate again to be in center of tile parking)
        trjJunctionTransition = drive.trajectorySequenceBuilder(trjScoreJunction.end())
            .lineTo(new Vector2d(-TILEx1_0 + 2, sideFlip(TILEx1_5 + 2.5)))
            .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                elevator.setWristOffset(0);
                elevator.setHandPosition(elevator.HAND_OPEN);
            })
            .splineToConstantHeading(new Vector2d(-TILEx0_5, sideFlip(TILEx2_0)), Math.toRadians(sideFlip(90)))
            .lineTo(new Vector2d(-TILEx0_5, sideFlip(TILEx2_0 + 8))) // center of scoring...
            .build();

        //=========================================================================================
        // Track to, and pickup cone from stack
        trjJunctionLoop1 = drive.trajectorySequenceBuilder(trjJunctionTransition.end())

            // Track cone and Grab cone when in position.
            .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                grabConeWhenReady = true;
                startConeTracking();
            })
            .waitSeconds(3)
            .build();

        //=========================================================================================
        // Back away from stack, turn to junction.  Track cone and drop Cone.
        trjJunctionLoop2 = drive.trajectorySequenceBuilder(new Pose2d(-TILEx0_5 , sideFlip(TILEx2_0 + 10), Math.toRadians(sideFlip(90))))
            // Back away from conestack and turn to junction.
            .setReversed(true)
            .lineTo(new Vector2d(-TILEx0_5, sideFlip(TILEx2_0 + 8)))
            .turn(Math.toRadians(sideFlip(120)))
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
        trjJunctionLoop3 = drive.trajectorySequenceBuilder(new Pose2d(-TILEx0_5 - 2, sideFlip(TILEx2_0 + 6), Math.toRadians(sideFlip(-130))))
            // Back away from junction and turn to cone-stack.
            .setReversed(true)
            .lineTo(new Vector2d(-TILEx0_5, sideFlip(TILEx2_0 + 8)))
            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                elevator.setWristOffset(0);
                elevator.setHandPosition(elevator.HAND_OPEN);
            })
            .turn(Math.toRadians(sideFlip(-130)))
            .setReversed(false)
            .build();

        //=========================================================================================
        trjStackPark1 = makeRedStackPark(1);
        trjStackPark2 = makeRedStackPark(2);
        trjStackPark3 = makeRedStackPark(3);

        trjJunctionPark1 = makeRedJunctionPark(1);
        trjJunctionPark2 = makeRedJunctionPark(2);
        trjJunctionPark3 = makeRedJunctionPark(3);

        // Drive straight to Park at Location 1
        trjSignalPark1 = makeRedSignalPark(1);
        trjSignalPark2 = makeRedSignalPark(2);
        trjSignalPark3 = makeRedSignalPark(3);
    }

    // ================================================================================================================
    //  TPARKING Builders...  RED FRONT / BLUE REAR
    // ================================================================================================================
    TrajectorySequence makeRedSignalPark(int signal) {
        TrajectorySequence tempSeq = drive.trajectorySequenceBuilder(trjReadSignal.end())
            // Make a run from the signal detection point
            .setVelConstraint(getVelocityConstraint(MAX_PUSH_VEL, MAX_ANG_VEL, TRACK_WIDTH))
            .lineTo(new Vector2d(-TILEx0_5 + 5, sideFlip(TILEx1_5)))
            .back(6)
            .lineTo(new Vector2d(-TILEx0_5 -1, parkingOffset(signal)))
            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { elevator.setWristOffset(0); })
            .UNSTABLE_addTemporalMarkerOffset(2.0, () -> { elevator.setLiftTargetPosition(Elevator.ELEVATOR_HOME); })
            .waitSeconds(4)
            .build();

        return tempSeq;
    }

    TrajectorySequence makeRedJunctionPark(int signal) {
        TrajectorySequence tempSeq = drive.trajectorySequenceBuilder(trjJunctionTransition.end())
            // Back away from junction and turn to cone-stack.
            .UNSTABLE_addTemporalMarkerOffset(0.0, () -> { elevator.setLiftTargetPosition(Elevator.ELEVATOR_LOW); })
            .waitSeconds(1)
            .lineTo(new Vector2d(-TILEx0_5, parkingOffset(signal)))
            .turn(Math.toRadians(sideFlip(-90)))
            .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                elevator.setLiftTargetPosition(Elevator.ELEVATOR_HOME);
            })
            .waitSeconds(2)
            .build();

        return tempSeq;
    }

    TrajectorySequence makeRedStackPark(int signal) {
        TrajectorySequence tempSeq = drive.trajectorySequenceBuilder(new Pose2d(-TILEx0_5 - 2, sideFlip(TILEx2_0 + 6), Math.toRadians(sideFlip(-130))))
            // Back away from junction and turn to cone-stack.
            .setReversed(true)
            .lineTo(new Vector2d(-TILEx0_5, parkingOffset(signal)))
            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { elevator.setHandPosition(elevator.HAND_OPEN); })
            .turn(Math.toRadians(sideFlip(-130)))
            .setReversed(false)
            .build();

        return tempSeq;
    }

    double parkingOffset(int signal) {
        if (coneStackOnLeft)
            return (TILEx1_5 - 1) - ((signal - 2) * TILEx1_0);
        else
            return (-TILEx1_5 + 1) - ((signal - 2) * TILEx1_0);
    }

    double sideFlip(double value) {
        if (coneStackOnLeft)
            return value;
        else
            return -value;
    }

}