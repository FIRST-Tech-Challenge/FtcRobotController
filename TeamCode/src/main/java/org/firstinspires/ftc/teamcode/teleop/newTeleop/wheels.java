package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "Wheel Control")
public class wheels extends LinearOpMode {
    
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() {
        // Initialize the OpMode
        initialize(); // Initialize hardware
        setDirection();// Set motor directions
        setBrakes();//Sets the motor brakes

        waitForStart(); // Wait for the start signal

        // Main loop for control   ing the robot during teleop
        while (opModeIsActive()) {
            finalMovement(); // Control robot movement
        }
    }
    
    public void initialize() {
        try {
            //DcMotor
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            telemetry.addLine("Initialization complete");
        } catch (NullPointerException e) {
            telemetry.addLine("Initialization error: " + e.getMessage());
            telemetry.update();
        } catch (IllegalArgumentException e) {
            telemetry.addLine("HardwareMap error: Check your motor/servo/sensor names");
            telemetry.addData("Error details", e.getMessage());
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("General initialization error: " + e.getMessage());
            telemetry.update();
        }
    }

    
    public void telemetryInit() {
        // Provide feedback about the robot's state
        while (opModeInInit()) {
            telemetry.addLine("=== Robot Initialization ===");
            telemetry.addLine("Status: Initializing");
            telemetry.addLine("Press START when ready");
            telemetry.update();
        }
    }

    
    public void setDirection() {
        // Set the direction of each motor
        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Reverse front left motor
        frontRight.setDirection(DcMotor.Direction.FORWARD); // Forward front right motor
        backLeft.setDirection(DcMotor.Direction.REVERSE); // Reverse back left motor
        backRight.setDirection(DcMotor.Direction.FORWARD);// Forward back right motor
        
    }

    
    public void setBrakes(){
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }

    public void movement(double vertical, double strafe, double turn) {
        // Set power to each motor based on gamepad input for movement
        frontLeft.setPower(-vertical - strafe - turn); // Calculate power for front left motor
        frontRight.setPower(-vertical + strafe + turn); // Calculate power for front right motor
        backLeft.setPower(-vertical + strafe - turn); // Calculate power for back left motor
        backRight.setPower(-vertical - strafe + turn); // Calculate power for back right motor
    }

    public void finalMovement() {
        double reduction = 0.8; // Default speed reduction
        double turnReduction = 0.55; // Default turning speed reduction

        // Adjust speeds based on button presses
        if (gamepad1.a) {
            // Slow mode
            reduction = 0.4;
            turnReduction = 0.35;
        } else if (gamepad1.b) {
            // Fast mode
            reduction = 1;
            turnReduction = 1;
        } else if ((gamepad1.left_stick_button) || (gamepad1.right_stick_button)) {
            // Stop mode
            reduction = 0.0;
            turnReduction = 0.0;
        }

        // Apply movement to motors based on gamepad input
        double vertical = reduction * gamepad1.left_stick_y; // Vertical movement
        double turn = -reduction * gamepad1.right_stick_x; // Turning movement
        double strafe = -turnReduction * gamepad1.left_stick_x; // Strafe movement
        movement(vertical, strafe, turn); // Call movement method with calculated powers
    }
}
