package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Vertical Slide Test", group = "Test")
public class VerticalSlideTest extends LinearOpMode {
    private DcMotorEx slideMotor;  // Using DcMotorEx for enhanced features
    private TouchSensor limitSwitch;
    
    // Constants for power control
    private static final double POWER_INCREMENT = 0.05;  // How much to change power per button press
    private static final double MAX_POWER = 0.5;        // Maximum allowed power
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        try {
            // Use DcMotorEx for better encoder access
            slideMotor = hardwareMap.get(DcMotorEx.class, "vertical_slide");
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            
            // Configure motor for GoBilda 5202 Yellow Jacket
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use RUN_WITHOUT_ENCODER for direct control
            
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Instructions", "Use triggers to control slide:");
            telemetry.addData("- Right Trigger", "Move Up (Negative Power)");
            telemetry.addData("- Left Trigger", "Move Down (Positive Power)");
            telemetry.addData("- Y Button", "Reset Encoder (if not at limit)");
            telemetry.addData("- B Button", "Stop Program");
            telemetry.addData("Motor Mode", slideMotor.getMode());
            telemetry.addData("Encoder Present", "Checking...");
            
            // Test if encoder is responding
            int startPos = slideMotor.getCurrentPosition();
            slideMotor.setPower(0.1);
            sleep(100);
            slideMotor.setPower(0);
            int endPos = slideMotor.getCurrentPosition();
            telemetry.addData("Encoder Test", startPos != endPos ? "WORKING" : "NOT RESPONDING");
            
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }
        telemetry.update();
        
        waitForStart();
        
        double currentPower = 0.0;
        int lastPosition = 0;
        
        while (opModeIsActive()) {
            // Get trigger values for power control
            double upPower = -gamepad1.right_trigger * MAX_POWER;    // Negative power = up
            double downPower = gamepad1.left_trigger * MAX_POWER;    // Positive power = down
            
            // Combine powers (only one trigger should be used at a time)
            currentPower = upPower + downPower;
            
            // Check limit switch before applying downward power
            if (limitSwitch.isPressed() && currentPower > 0) {
                // If limit switch is pressed and trying to go down, stop and reset encoder
                currentPower = 0;
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            
            // Manual encoder reset if needed (useful for testing)
            if (gamepad1.y) {
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            
            // Apply power to motor
            slideMotor.setPower(currentPower);
            
            // Get current position and calculate change
            int currentPosition = slideMotor.getCurrentPosition();
            int positionChange = currentPosition - lastPosition;
            lastPosition = currentPosition;
            
            // Display current state
            telemetry.addData("Encoder Position", currentPosition);
            telemetry.addData("Position Change", positionChange);
            telemetry.addData("Current Power", "%.2f", currentPower);
            telemetry.addData("Limit Switch", limitSwitch.isPressed() ? "PRESSED" : "NOT PRESSED");
            telemetry.addData("Motor Mode", slideMotor.getMode());
            telemetry.addData("Controls", "RT = Up, LT = Down, Y = Reset Encoder");
            telemetry.update();
            
            // Exit if B is pressed
            if (gamepad1.b) {
                break;
            }
        }
        
        // Stop motor when done
        slideMotor.setPower(0);
    }
} 