/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "HuskyLens Blue Face Detection", group = "Linear Opmode")
public class HuskyLensBlue extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        // Initialize the motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Set motors to brake when power is set to zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "husky_lens");

        // Wait for start button to be pressed
        telemetry.addData("Status", "Initialized and waiting for start");
        telemetry.update();
        waitForStart();

        // Start strafing left until detection occurs
        while (opModeIsActive()) {
            // Update HuskyLens data
            huskyLens.getClass();

            // Check if HuskyLens has detected any objects
            if (huskyLens.hasLearned()) {
                // Get all detected blocks
                HuskyLens.Block block = huskyLens.getBlockByID(1); // Adjust ID based on the trained model

                if (block != null && block.label.equals("blue-face-of-piece")) {
                    // Object detected, stop the robot
                    stopRobot();

                    telemetry.addData("Detection", "Blue face detected, stopping robot.");
                    telemetry.update();
                    break; // Exit the loop after detection
                } else {
                    // No detection, continue strafing left
                    strafeLeft();
                    telemetry.addData("Detection", "Blue face not detected, strafing left.");
                    telemetry.update();
                }
            } else {
                telemetry.addData("Status", "No objects detected.");
                telemetry.update();
            }
        }
    }

    // Method to stop the robot
    private void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // Method to strafe the robot left
    private void strafeLeft() {
        frontLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(-0.5);
    }
}}*/