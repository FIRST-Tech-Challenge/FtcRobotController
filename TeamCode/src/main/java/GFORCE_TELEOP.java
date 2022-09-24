import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize SampleMecanumDrive
        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // reset heading if double button press
            if (gamepad1.back && gamepad1.start) {
                drive.setPoseEstimate( new Pose2d(poseEstimate.getX(), poseEstimate.getY(), 0));
                drive.setExternalHeading(0);
            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // are we turning or should heading be locked.
            if (Math.abs(gamepad1.right_stick_x) < 0.01) {
                if (!headingLock) {
                    headingLock = true;
                    headingSetpoint = poseEstimate.getHeading();
                }
            } else {
                headingLock = false;
            }

            if (headingLock) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x / 10
                        )
                );
            } else {
                // Pass in the rotated input + right stick value for rotation
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x / 10
                        )
                );
            }

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("Lock", headingLock);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("ODO  heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();
        }
    }
}
