package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Claw and Slide Test", group = "Test")
public class ClawServoTest extends LinearOpMode {
    private Claw claw;
    private DcMotorEx slideMotor;  // Direct motor control for testing
    private TouchSensor limitSwitch;
    
    // Constants for slide control
    private static final double MAX_POWER = 0.5;
    private static final double HOLDING_POWER = 0.1;  // Small negative power to hold against gravity
    
    @Override
    public void runOpMode() {
        // Initialize claw system
        try {
            claw = new Claw(hardwareMap);
            telemetry.addData("Status", "Claw system initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not initialize claw system: " + e.getMessage());
        }
        
        // Initialize direct motor control for testing
        try {
            slideMotor = hardwareMap.get(DcMotorEx.class, "vertical_slide");
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            
            // Configure motor for GoBilda 5202 Yellow Jacket
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            telemetry.addData("Slide Status", "Initialized");
            telemetry.addData("Motor Mode", slideMotor.getMode());
            
            // Test encoder
            int startPos = slideMotor.getCurrentPosition();
            slideMotor.setPower(0.1);
            sleep(100);
            slideMotor.setPower(0);
            int endPos = slideMotor.getCurrentPosition();
            telemetry.addData("Encoder Test", startPos != endPos ? "WORKING" : "NOT RESPONDING");
        } catch (Exception e) {
            telemetry.addData("Slide Error", e.getMessage());
        }
        
        telemetry.addData("Instructions", "Use gamepad1 buttons to test servos:");
        telemetry.addData("Claw", "A: Open, B: Close");
        telemetry.addData("Wrist", "X: Up, Y: Down");
        telemetry.addData("Elbow", "DPad_Up: Up, DPad_Right: Forward, DPad_Down: Down");
        telemetry.addData("Manual", "Hold Right Bumper + Left Bumper/X/Y for fine control");
        telemetry.addData("Slide", "RT = Up, LT = Down (Direct Motor Control)");
        telemetry.update();
        
        waitForStart();
        
        double currentPower = 0.0;
        int lastPosition = 0;
        
        while (opModeIsActive() && claw != null) {
            // Test claw servo
            if (gamepad1.a) {
                claw.openClaw();
                telemetry.addData("Claw", "Opening");
            } else if (gamepad1.b) {
                claw.closeClaw();
                telemetry.addData("Claw", "Closing");
            }
            
            // Test wrist servo
            if (gamepad1.x) {
                claw.wristUp();
                telemetry.addData("Wrist", "Moving Up");
            } else if (gamepad1.y) {
                claw.wristDown();
                telemetry.addData("Wrist", "Moving Down");
            }
            
            // Test elbow servo
            if (gamepad1.dpad_up) {
                claw.elbowUp();
                telemetry.addData("Elbow", "Moving Up");
            } else if (gamepad1.dpad_right) {
                claw.elbowForward();
                telemetry.addData("Elbow", "Moving Forward");
            } else if (gamepad1.dpad_down) {
                claw.elbowDown();
                telemetry.addData("Elbow", "Moving Down");
            }
            
            // Manual position control with bumpers and triggers
            if (gamepad1.right_bumper) {
                if (gamepad1.left_bumper) {
                    // Manual claw control
                    double pos = claw.getClawPosition();
                    pos += gamepad1.right_trigger * 0.01;
                    pos -= gamepad1.left_trigger * 0.01;
                    pos = Math.min(Math.max(pos, 0), 1);
                    claw.setClawPosition(pos);
                    telemetry.addData("Manual Claw Pos", "%.2f", pos);
                } else if (gamepad1.x) {
                    // Manual wrist control
                    double pos = claw.getWristPosition();
                    pos += gamepad1.right_trigger * 0.01;
                    pos -= gamepad1.left_trigger * 0.01;
                    pos = Math.min(Math.max(pos, 0), 1);
                    claw.setWristPosition(pos);
                    telemetry.addData("Manual Wrist Pos", "%.2f", pos);
                } else if (gamepad1.y) {
                    // Manual elbow control
                    double pos = claw.getElbowPosition();
                    pos += gamepad1.right_trigger * 0.01;
                    pos -= gamepad1.left_trigger * 0.01;
                    pos = Math.min(Math.max(pos, 0), 1);
                    claw.setElbowPosition(pos);
                    telemetry.addData("Manual Elbow Pos", "%.2f", pos);
                } else {
                    // Direct slide control with triggers when right bumper is held alone
                    double upPower = -gamepad1.right_trigger * MAX_POWER;    // Negative = up
                    double downPower = gamepad1.left_trigger * MAX_POWER;    // Positive = down
                    currentPower = upPower + downPower;
                    
                    // Check limit switch
                    if (limitSwitch.isPressed() && currentPower > 0) {
                        currentPower = 0;
                        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    
                    // Apply power, using holding power when no input
                    if (Math.abs(currentPower) < 0.01) {
                        slideMotor.setPower(HOLDING_POWER);  // Apply holding power when no input
                    } else {
                        slideMotor.setPower(currentPower);
                    }
                    
                    // Track position changes
                    int currentPosition = slideMotor.getCurrentPosition();
                    int positionChange = currentPosition - lastPosition;
                    lastPosition = currentPosition;
                    
                    telemetry.addData("Manual Slide Control", "Active");
                    telemetry.addData("Power", "%.2f", currentPower);
                    telemetry.addData("Position Change", positionChange);
                }
            } else {
                // When not in manual control, apply holding power
                slideMotor.setPower(HOLDING_POWER);
            }
            
            // Display current positions
            telemetry.addData("Claw Position", "%.2f", claw.getClawPosition());
            telemetry.addData("Wrist Position", "%.2f", claw.getWristPosition());
            telemetry.addData("Elbow Position", "%.2f", claw.getElbowPosition());
            telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
            telemetry.addData("Limit Switch", limitSwitch.isPressed() ? "PRESSED" : "NOT PRESSED");
            telemetry.update();
        }
    }
} 