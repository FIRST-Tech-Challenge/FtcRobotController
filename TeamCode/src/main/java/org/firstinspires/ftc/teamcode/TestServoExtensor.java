package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Servo Positions", group = "TeleOp")
public class TestServoExtensor extends LinearOpMode {
    private Servo wrist;
    private Servo clawL;
    private Servo clawR;
    
    // Amount to change servo position per button press
    private final double SERVO_SPEED = 0.05;
    
    @Override
    public void runOpMode() {
        // Initialize servos
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        
        // Wait for the game pad to start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
            // Control wrist with bumpers
            if (gamepad1.right_bumper) {
                wrist.setPosition(wrist.getPosition() + SERVO_SPEED);
            }
            if (gamepad1.left_bumper) {
                wrist.setPosition(wrist.getPosition() - SERVO_SPEED);
            }
            
            // Control left claw with dpad
            if (gamepad1.dpad_right) {
                clawL.setPosition(clawL.getPosition() + SERVO_SPEED);
            }
            if (gamepad1.dpad_left) {
                clawL.setPosition(clawL.getPosition() - SERVO_SPEED);
            }
            
            // Control right claw with X and B buttons
            if (gamepad1.b) {
                clawR.setPosition(clawR.getPosition() + SERVO_SPEED);
            }
            if (gamepad1.x) {
                clawR.setPosition(clawR.getPosition() - SERVO_SPEED);
            }
            
            // Display current positions
            telemetry.addData("Wrist Position", "%.3f", wrist.getPosition());
            telemetry.addData("Left Claw Position", "%.3f", clawL.getPosition());
            telemetry.addData("Right Claw Position", "%.3f", clawR.getPosition());
            telemetry.update();
            
            // Small delay to prevent the loop from running too fast
            sleep(50);
        }
    }
}
