package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad;
import org.firstinspires.ftc.teamcode.SubSystems.HzVuforia;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;


/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@Autonomous(name = "Hazmat Red Autonomous Outer", group = "00-Autonomous")
public class HazmatRedAutonomousOuter extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepad hzGamepad1;
    SampleMecanumDrive drive;
    HzVuforia hzVuforia1;
    int playingAlliance = 1; //1 for Red, -1 for Blue
    int startLine = 1 ; //0 for inner, 1 for outer
    boolean parked = false ;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        drive = new SampleMecanumDrive(hardwareMap);
        hzGamepad1 = new HzGamepad(gamepad1);
        hzVuforia1 = new HzVuforia(hardwareMap);

        // Set initial pose
        if (playingAlliance == 1) /* Red */ {
            if (startLine == 0)  /* Inner Line */ {
                drive.setPoseEstimate(new Pose2d(-68,-24,Math.toRadians(0))); // Red Inner Start Line
            } else /* Outer Line */ {
                drive.setPoseEstimate(new Pose2d(-68,-48,Math.toRadians(0))); // Red Outer Start Line
            }
        } else /* Blue */ {
            if (startLine == 0) /* Inner Line */{
                drive.setPoseEstimate(new Pose2d(-68,24,Math.toRadians(0))); // Blue Inner Start Line
            } else /* Outer Line */ {
                drive.setPoseEstimate(new Pose2d(-68,48,Math.toRadians(0))); // Blue Outer Start Line
            }
        }

        // Initiate Camera even before Start is pressed.
        //waitForStart();

        hzVuforia1.activateVuforiaTensorFlow();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Init is pressed at this time, and start is not pressed yet

            //Run Vuforia Tensor Flow
            hzVuforia1.runVuforiaTensorFlow();


            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive() && !parked) {

                hzVuforia1.deactivateVuforiaTensorFlow();

                // Example spline path from SplineTest.java
                // Make sure the start pose matches with the localizer's start pose
                Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(45, 45), 0)
                        .build();

                drive.followTrajectory(traj);

                sleep(2000);

                drive.followTrajectory(
                        drive.trajectoryBuilder(traj.end(), true)
                                .splineTo(new Vector2d(15, 15), Math.toRadians(180))
                                .build()
                );

                parked = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        hzVuforia1.deactivateVuforiaTensorFlow();
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        // Print pose to telemetry
        telemetry.addData("PoseEstimate : x", hzGamepad1.poseEstimate.getX());
        telemetry.addData("PoseEstimate : y", hzGamepad1.poseEstimate.getY());
        telemetry.addData("PoseEstimate : heading", Math.toDegrees(hzGamepad1.poseEstimate.getHeading()));

        telemetry.addData("Visible Target : ", hzVuforia1.visibleTargetName);
        // Print pose to telemetry
        telemetry.addData("PoseVuforia : x", hzVuforia1.poseVuforia.getX());
        telemetry.addData("PoseVuforia : y", hzVuforia1.poseVuforia.getY());
        telemetry.addData("PoseVuforia : heading", Math.toDegrees(hzVuforia1.poseVuforia.getHeading()));

        telemetry.update();

    }
}
