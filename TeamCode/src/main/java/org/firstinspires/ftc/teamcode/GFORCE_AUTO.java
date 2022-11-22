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
    AutoConfig autoConfig = new AutoConfig();
    boolean weAreRed;
    GFORCE_KiwiDrive drive = null;
    boolean trackConeNow = false;
    boolean coneGrabbed = false;

    Double ONE_TILE = 23.5;

    // declare all trajectories //
    TrajectorySequence redFrontJunctionInit;
    TrajectorySequence redFrontJunctionLoop;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize GFORCE_KiwiDrive
        drive = new GFORCE_KiwiDrive(hardwareMap);
        autoConfig.init(this);
        elevator = new Elevator(this, true);
        coneTracker = new ConeTracker(this);


        while (opModeInInit()) {
            elevator.update();
            elevator.runStateMachine();
            autoConfig.init_loop();
            //read signal cone
            // Select the desired trajectory
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
                            telemetry.addData("autorunning","first sequence finish");
                            telemetry.update();
                            trackConeNow = false;
                            followGforceSequence(redFrontJunctionLoop);
                            telemetry.addData("autorunning","second sequence finish");
                            telemetry.update();
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


    public void followGforceSequence(TrajectorySequence trajectory) {
        drive.followTrajectorySequenceAsync(trajectory);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            elevator.update();
            elevator.runStateMachine();
            if (!trackConeNow) {
                drive.update();
            } else {
                coneHoming();
            }
        }
    }

    public void coneHoming() {
        drive.updatePoseEstimate();
        if (coneTracker.update() && !coneGrabbed) {
            coneTracker.showRanges();
            drive.setWeightedDrivePower(new Pose2d(coneTracker.trackDrive(), 0, coneTracker.trackTurn()));

            if (coneTracker.trackGrab()) {
                elevator.grabRequest = true;
                coneGrabbed = true;
                trackConeNow = false;
            }
        } else {
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        }
        telemetry.update();
    }

    public void startConeTracking() {
        trackConeNow = true;
        coneGrabbed = false;
    }

    // trajectory builders //
    private void buildRedFrontJunction (){
        drive.setExternalHeading(Math.toRadians(0));
        Pose2d startPosition = new Pose2d(new Vector2d(-62, 35), Math.toRadians(0));
        drive.setPoseEstimate(startPosition);
        redFrontJunctionInit = drive.trajectorySequenceBuilder(startPosition)
            .addDisplacementMarker(0.5, () -> {
                elevator.levelUp();
            })
            .splineTo(new Vector2d(-53, 30), Math.toRadians(-45))
            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                elevator.autoRelease();
            })
            .waitSeconds(1.5)
            .setReversed(true)
            .splineTo(new Vector2d(-59, 35), Math.toRadians(-90))
            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                elevator.setLiftTargetPosition(Elevator.ELEVATOR_STACK_TOP);
            })
            .lineTo(new Vector2d((ONE_TILE * -1.0), (ONE_TILE * 1.5)))
            .splineToConstantHeading(new Vector2d((ONE_TILE * -0.5), 51), Math.toRadians(90))
            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                elevator.setWristOffset(0);
            })
            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
              startConeTracking();
            })
            .waitSeconds(1.5)
            .build();

           startPosition = new Pose2d(new Vector2d((ONE_TILE * -0.5), (ONE_TILE * 2.5)), Math.toRadians(90));
           redFrontJunctionLoop =drive.trajectorySequenceBuilder(startPosition)
           .back(3)
           .setReversed(false)
           .splineTo(new Vector2d((7.6 - ONE_TILE ), (2 * ONE_TILE + 6.7)),Math.toRadians(-135))
           .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
               elevator.autoRelease();
           })
           .waitSeconds(1.5)
           .back(2)
           .setReversed(false)
           .splineToLinearHeading(new Pose2d((ONE_TILE * -0.5), 51, Math.toRadians(90)), Math.toRadians(90))
                   .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                       elevator.setLiftTargetPosition((Elevator.ELEVATOR_STACK_TOP)-20);
                       elevator.setWristOffset(0);
                       elevator.setHandPosition(elevator.HAND_OPEN);
                       startConeTracking();
                   })
                   .waitSeconds(1.5)
           .build();
    }
}
