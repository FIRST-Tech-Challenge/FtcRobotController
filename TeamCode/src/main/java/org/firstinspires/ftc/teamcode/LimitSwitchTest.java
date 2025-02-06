package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Limit Switch Test", group = "Test")
public class LimitSwitchTest extends LinearOpMode {
    private TouchSensor limitSwitch;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        
        // Try to get the limit switch configuration
        try {
            // List all configured devices
            telemetry.addData("Status", "Checking configuration...");
            telemetry.addData("Looking for Touch Sensors", "");
            for (String name : hardwareMap.getAllNames(TouchSensor.class)) {
                telemetry.addData("Found Touch Sensor", name);
            }
            telemetry.update();
            
            // Try to initialize the limit switch
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            telemetry.addData("Status", "Found limit switch");
            
            // Try to get more information about the sensor
            if (limitSwitch instanceof DigitalChannel) {
                telemetry.addData("Sensor Type", "Digital Channel");
                DigitalChannel digitalSwitch = (DigitalChannel)limitSwitch;
                digitalSwitch.setMode(DigitalChannel.Mode.INPUT);
                telemetry.addData("Digital Mode", "Set to INPUT");
            } else {
                telemetry.addData("Sensor Type", "Standard Touch Sensor");
            }
            
            telemetry.addData("Connection Info", "Make sure:");
            telemetry.addData("- Signal (White)", "Connected to S pin");
            telemetry.addData("- Power (Red)", "Connected to V pin (5V)");
            telemetry.addData("- Ground (Black)", "Connected to G pin");
            telemetry.update();
            
            telemetry.addData("Status", "Ready!");
            telemetry.addData("Instructions", "Press Start to begin testing");
            telemetry.addData("", "The switch state will be displayed continuously");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize: " + e.getMessage());
            telemetry.addData("", "Check that:");
            telemetry.addData("1.", "Device name is exactly 'limit_switch'");
            telemetry.addData("2.", "Device is configured as REV Touch Sensor");
            telemetry.addData("3.", "Wire connections are secure");
        }
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive() && limitSwitch != null) {
            boolean isPressed = limitSwitch.isPressed();
            double value = limitSwitch.getValue();
            
            telemetry.addData("Switch State", isPressed ? "PRESSED" : "NOT PRESSED");
            telemetry.addData("Raw Value", "%.3f", value);
            
            // Additional debugging info
            if (limitSwitch instanceof DigitalChannel) {
                DigitalChannel digitalSwitch = (DigitalChannel)limitSwitch;
                telemetry.addData("Digital State", digitalSwitch.getState() ? "HIGH" : "LOW");
            }
            
            telemetry.addData("", "Press B to quit");
            if (gamepad1.b) {
                break;
            }
            
            telemetry.update();
            sleep(50); // Small delay to prevent flooding telemetry
        }
    }
} 