package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HEADING_PID;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.GFORCE_KiwiDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name="G-FORCE TELEOP", group = "advanced")
public class GFORCE_TELEOP extends LinearOpMode {

    boolean headingLock = false;
    double  headingSetpoint = 0;

    double AXIAL_RATE = 0.8;
    double LATERAL_RATE = 0.8;
    double YAW_RATE = 0.8;


    private Elevator elevator;


    // Declare a PIDF Controller to regulate heading
    // Use the same gains as GFORCE_KiwiDrive's heading controller
    private PIDFController headingController = new PIDFController(HEADING_PID);

    @Override
    public void runOpMode() throws InterruptedException {

        double joysticRotate;

        headingController.setInputBounds(0.0, 2.0 * Math.PI);

        // Initialize GFORCE_KiwiDrive
        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);

        elevator = new Elevator(this);

        elevator.resetHome();

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Update everything.
            drive.update();

            //-----------PILOT-----------
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of the heading
            Vector2d joysticInput = new Vector2d(
                    -gamepad1.left_stick_y * LATERAL_RATE,
                    -gamepad1.right_stick_x * AXIAL_RATE
            ).rotated(-poseEstimate.getHeading());

            // Determine the required rotate rate
            joysticRotate = (gamepad1.left_trigger - gamepad1.right_trigger) * YAW_RATE * DriveConstants.kV * 5  ;

            //set one of four desired headings
            if(gamepad1.triangle) {
                lockNewHeading(Math.toRadians(0));
            }else if (gamepad1.circle) {
                lockNewHeading(Math.toRadians(270));
            }else if (gamepad1.cross) {
                lockNewHeading(Math.toRadians(180));
            }else if (gamepad1.square) {
                lockNewHeading(Math.toRadians(90));
            }

            // are we turning or should heading be locked.
            if (Math.abs(joysticRotate) < 0.01) {
                if (!headingLock && drive.notTurning()) {
                    lockNewHeading(drive.getExternalHeading());
                }
            } else {
                headingLock = false;
            }

            if (headingLock) {
                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = headingController.update(drive.getExternalHeading())
                        * DriveConstants.kV ;

                drive.setWeightedDrivePower(
                        new Pose2d(
                                joysticInput.getX(),
                                joysticInput.getY(),
                                headingInput
                        )
                );
            } else {
                // Pass in the rotated input + right stick value for rotation
                drive.setWeightedDrivePower(
                        new Pose2d(
                                joysticInput.getX(),
                                joysticInput.getY(),
                                joysticRotate
                        )
                );
            }
            // reset heading if double button press
            if (gamepad1.back && gamepad1.start) {
                drive.setPoseEstimate( new Pose2d() );
                drive.setExternalHeading(0);
                headingSetpoint = 0;
                headingController.setTargetPosition(headingSetpoint);
            }



            //-----------CO-PILOT--------------
            //Homes elevator
            if (gamepad2.left_bumper) {
                elevator.setHandPosition(elevator.HAND_CLOSE);
                elevator.setTarget(elevator.ELEVATOR_HOME);
            }

            //elevator.manualControl();

            if (gamepad2.dpad_down) {
                elevator.setTarget(elevator.ELEVATOR_LOW);
            } else if (gamepad2.dpad_left) {
                elevator.setTarget(elevator.ELEVATOR_MID);
            } else if (gamepad2.dpad_up) {
                elevator.setTarget(elevator.ELEVATOR_HIGH);
            } else if (gamepad2.dpad_right) {
                elevator.setTarget(elevator.ELEVATOR_GROUND);
            }

            if (gamepad2.circle) {
                elevator.setHandPosition(elevator.HAND_OPEN);
                elevator.setWristOffset(0);
            } else if(gamepad2.square) {
                elevator.setHandPosition(elevator.HAND_CLOSE);
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                elevator.setWristOffset(90);
            }

            elevator.jogElevator(-gamepad2.left_stick_y);


            elevator.runControlLoop();

            telemetry.addData("Lock", headingLock);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("ODO  heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.addData ("Elevator position", elevator.getPosition());
            telemetry.addData ("arm position", elevator.getPosition());
            telemetry.update();
        }
    }

    public void lockNewHeading(double heading) {
        headingLock = true;
        headingSetpoint = heading;
        headingController.setTargetPosition(headingSetpoint);
    }

}
