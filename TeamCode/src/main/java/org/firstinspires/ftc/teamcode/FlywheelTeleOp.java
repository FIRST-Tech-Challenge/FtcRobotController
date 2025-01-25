package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Flywheel TeleOp", group = "Test")
public class FlywheelTeleOp extends LinearOpMode {
    private CRServo flywheel;
    
    // Constants for power settings
    private static final double FULL_POWER = 1.0;
    private static final double STOP_POWER = 0.0;
    
    @Override
    public void runOpMode() {
        // Get the flywheel from hardware map
        flywheel = hardwareMap.get(CRServo.class, "flywheel");
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "RT = Forward, LT = Reverse");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Get trigger values
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.1;
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.1;
            
            // Set power based on triggers - using full power for testing
            if (rightTriggerPressed) {
                flywheel.setPower(FULL_POWER);  // Full power forward
            } else if (leftTriggerPressed) {
                flywheel.setPower(-FULL_POWER); // Full power reverse
            } else {
                flywheel.setPower(STOP_POWER);  // Stop
            }
            
            // Display current state
            telemetry.addData("Power Setting", "%.2f", flywheel.getPower());
            telemetry.addData("Right Trigger Pressed", rightTriggerPressed);
            telemetry.addData("Left Trigger Pressed", leftTriggerPressed);
            telemetry.update();
        }
    }
} 