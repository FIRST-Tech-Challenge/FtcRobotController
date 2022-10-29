package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    // Joystick constants
    final double AXIAL_RATE = 0.7;
    final double LATERAL_RATE = 0.7;
    final double YAW_RATE = 0.5;

    private Elevator    elevator;
    private ConeTracker coneTracker;

    boolean headingLock = false;
    double  headingSetpoint = 0;
    boolean lastFlip = false;
    boolean flip = false;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as GFORCE_KiwiDrive's heading controller
    private PIDFController headingController = new PIDFController(new PIDCoefficients(1.25, 0, 0));

    @Override
    public void runOpMode() throws InterruptedException {

        // apply hub performance
        HubPerformance.enable(hardwareMap);

        double manualRotate;

        headingController.setInputBounds(0.0, 2.0 * Math.PI);
        headingController.setOutputBounds(-YAW_RATE, YAW_RATE);

        // Initialize robot hardware classes GFORCE_KiwiDrive
        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);
        elevator = new Elevator(this);
        coneTracker = new ConeTracker(this);

        // Home the elevator....  This may need to be changed once we have an Auto.
        telemetry.addData("Elevator", "Homing");
        telemetry.update();
        elevator.recalibrateHomePosition();
        telemetry.addData("Elevator", "Homed !");
        telemetry.update();

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        while (opModeIsActive()) {

            // Update everything.
            drive.update();
            elevator.update();
            coneTracker.update();
            coneTracker.showRanges();

            //-----------PILOT-----------
            //check for auto cone tracking

            if (gamepad1.left_bumper && coneTracker.coneDetected) {
                double turn = coneTracker.coneDirection / 5.0;
                double speed = 0;

                if (coneTracker.coneRange > 100) {
                    speed = 0.2;
                }else if (coneTracker.coneRange >85) {
                    speed = 0.1;
                }else if (coneTracker.coneRange < 65) {
                    speed = -0.05;
                }

                lockNewHeading(drive.getExternalHeading());
                drive.setWeightedDrivePower(new Pose2d(speed, 0, turn));

            } else  {
                // Read pose and use it to convery joystick inputs to Field Centric.
                Pose2d poseEstimate = drive.getPoseEstimate();

                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of the heading
                Vector2d joysticInput = new Vector2d(
                        -gamepad1.left_stick_y * LATERAL_RATE,
                        -gamepad1.right_stick_x * AXIAL_RATE
                ).rotated(-poseEstimate.getHeading());

                // Determine the rotate rate being requested by pilot.
                manualRotate = (gamepad1.left_trigger - gamepad1.right_trigger) * YAW_RATE;

                // also check to see if the pilot is requesting a spin to one of the XY axes
                if (gamepad1.triangle) {
                    lockNewHeading(Math.toRadians(0));
                } else if (gamepad1.circle) {
                    lockNewHeading(Math.toRadians(270));
                } else if (gamepad1.cross) {
                    lockNewHeading(Math.toRadians(180));
                } else if (gamepad1.square) {
                    lockNewHeading(Math.toRadians(90));
                }

                // are we turning or should heading be locked.
                drive.notTurning();
                if (Math.abs(manualRotate) < 0.01) {
                    if (!headingLock && drive.notTurning()) {
                        lockNewHeading(drive.getExternalHeading());
                    }
                } else {
                    headingLock = false;
                }

                if (headingLock) {
                    // Set desired angular velocity to the heading-controller output
                    double autoRotate = headingController.update(drive.getExternalHeading())  ;

                    telemetry.addData("Auto Powers", "%.2f, %.2f, %.2f", joysticInput.getX(),joysticInput.getY(), autoRotate);

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    joysticInput.getX(),
                                    joysticInput.getY(),
                                    autoRotate
                            )
                    );
                } else {
                    // Pass in the rotated input + right stick value for rotation

                    telemetry.addData("Manual Powers", "%.2f, %.2f, %.2f", joysticInput.getX(),joysticInput.getY(), manualRotate);
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    joysticInput.getX(),
                                    joysticInput.getY(),
                                    manualRotate
                            )
                    );
                }
            }
            // reset heading if double button press
            if (gamepad1.back && gamepad1.start) {
                drive.setPoseEstimate( new Pose2d() );
                drive.setExternalHeading(0);
                headingSetpoint = 0;
                headingController.setTargetPosition(headingSetpoint);
            }

            //-----------CO-PILOT--------------
            // Lower the elevator to the base

            if (gamepad2.left_bumper) {
                //elevator.setHandPosition(elevator.HAND_CLOSE);//
                elevator.setLiftTargetPosition(elevator.ELEVATOR_HOME);
            }

            // Select one of the 4 preset heights
            if (gamepad2.dpad_down) {
                elevator.setLiftTargetPosition(elevator.ELEVATOR_LOW);
            } else if (gamepad2.dpad_left) {
                elevator.setLiftTargetPosition(elevator.ELEVATOR_MID);
            } else if (gamepad2.dpad_up) {
                elevator.setLiftTargetPosition(elevator.ELEVATOR_HIGH);
            } else if (gamepad2.dpad_right) {
                elevator.setLiftTargetPosition(elevator.ELEVATOR_GROUND);
            }

            // Open or close the hand (gripper)
            if (gamepad2.circle) {
                elevator.setHandPosition(elevator.HAND_OPEN);
                elevator.setWristOffset(0);
            } else if(gamepad2.square) {
                elevator.setHandPosition(elevator.HAND_CLOSE);
            }

            // Put the hand in safe position
            flip = (gamepad1.right_bumper || gamepad2.right_bumper);
            if (flip && !lastFlip) {
                if (elevator.getWristOffset() == elevator.SAFE_WRIST_OFFSET) {
                   elevator.setWristOffset(0);
                } else {
                    elevator.setWristOffset(elevator.SAFE_WRIST_OFFSET);
                }
            }
            lastFlip = flip;

            // Manually jog the elevator.
            elevator.jogElevator(-gamepad2.left_stick_y);

            // Display Telemetry data
            telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();
        }
    }

    // Lock in a new heading for the Auto Heading Hold..
    public void lockNewHeading(double heading) {
        headingLock = true;
        headingSetpoint = heading;
        headingController.setTargetPosition(headingSetpoint);
    }
}
