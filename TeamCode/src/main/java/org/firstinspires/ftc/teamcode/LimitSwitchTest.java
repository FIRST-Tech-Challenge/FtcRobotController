package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

@TeleOp(name = "Limit Switch Test", group = "Test")
public class LimitSwitchTest extends LinearOpMode {
    private TouchSensor limitSwitch;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        
        try {
            // Initialize the limit switch
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            telemetry.addData("Status", "Found limit switch");
            telemetry.addData("Sensor Class", limitSwitch.getClass().getName());
            telemetry.addData("Initial State", limitSwitch.isPressed() ? "PRESSED" : "NOT PRESSED");
            
            telemetry.addData("", "=== WIRING CHECK ===");
            telemetry.addData("1. Signal Wire (White)", "→ S pin");
            telemetry.addData("2. Power Wire (Red)", "→ V (5V) pin");
            telemetry.addData("3. Ground Wire (Black)", "→ G pin");
            telemetry.addData("", "When pressed, switch should connect Signal to Ground");
            telemetry.addData("", "If LED is on but switch never reads PRESSED:");
            telemetry.addData("", "1. Check signal wire connection");
            telemetry.addData("", "2. Verify switch is configured as 'REV Touch Sensor'");
            telemetry.addData("", "=================");
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize: " + e.getMessage());
        }
        telemetry.update();
        
        // Wait for start
        telemetry.addData("", "Press START to begin testing");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive() && limitSwitch != null) {
            boolean isPressed = limitSwitch.isPressed();
            
            telemetry.addData("=== SENSOR STATE ===", "");
            telemetry.addData("Pressed", isPressed ? "YES" : "NO");
            telemetry.addData("Sensor Type", limitSwitch.getClass().getSimpleName());
            telemetry.addData("", "Press B to quit");
            telemetry.update();
            
            if (gamepad1.b) break;
            sleep(50);
        }
    }
} 