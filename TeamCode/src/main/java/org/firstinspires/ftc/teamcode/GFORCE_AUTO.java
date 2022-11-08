package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@Autonomous(name="G-FORCE AUTO", group = "advanced")
public class GFORCE_AUTO extends LinearOpMode {

    private Elevator    elevator;
    private ConeTracker coneTracker;
    AutoConfig autoConfig = new AutoConfig();
    boolean weAreRed;
    GFORCE_KiwiDrive drive = null;


    @Override
    public void runOpMode() throws InterruptedException{

        // Initialize GFORCE_KiwiDrive
        drive = new GFORCE_KiwiDrive(hardwareMap);
        autoConfig.init(this);
        elevator = new Elevator(this);
        coneTracker = new ConeTracker(this);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        // drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details


        TrajectorySequence ourTrajectory = null;

        while (opModeInInit()) {
            autoConfig.init_loop();
            elevator.update();
            elevator.runStateMachine();
            //read signal cone
            // Select the desired trajectory
        }



        if (opModeIsActive()) {
            if (autoConfig.autoOptions.enabled) {
                weAreRed = autoConfig.autoOptions.redAlliance;
                if (weAreRed) {
                    if (autoConfig.autoOptions.startFront) {
                        Pose2d startPosition = new Pose2d(new Vector2d(-35,-62), Math.toRadians(90));
                        drive.setExternalHeading(Math.toRadians(90));
                        drive.setPoseEstimate(startPosition);
                        if(autoConfig.autoOptions.scoreJunction) {
                            //we are red, starting at the front, scoring the junction

                            ourTrajectory = drive.trajectorySequenceBuilder(startPosition)
                                    .addDisplacementMarker(2, () -> {
                                    elevator.levelUp();
                                    })
                                    .splineTo(new Vector2d(-30, -53), Math.toRadians(45))
                                    .addDisplacementMarker(() -> {
                                      elevator.autoRelease();
                                    })
                                    .waitSeconds(2)
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-35, -59), Math.toRadians(0))
                            .build();
                            followGforceSequence(ourTrajectory);
                        }

                    }
                }
            }

        }

        // save the last field position in shared states class for teleop to use.
        SharedStates.currentPose = drive.getPoseEstimate();
    }
    public void followGforceSequence(TrajectorySequence trajectory) {
        drive.followTrajectorySequenceAsync(trajectory);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            elevator.update();
            elevator.runStateMachine();
            drive.update();
        }
    }
}
