package org.firstinspires.ftc.teamcode.functions;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BadServoFunctions {
        public Servo wristServo;
        public Servo clawServo;
        // Variables to store positions for wrist and claw

        double wristPosition = 0.5; // Starting position for wrist
        double clawPosition = 0.5; // Starting position for claw

    public BadServoFunctions(HardwareMap hardwareMap){
        wristServo=hardwareMap.get(Servo.class,"wrist_y");
        clawServo=hardwareMap.get(Servo.class,"claw");
    }
        // Method to control the wrist position
        public void controlWrist (Gamepad gamepad1, Telemetry telemetry){
            double wristAdjustment = 0.01; // Tune this for smooth movement

            // Adjust wrist position based on gamepad input
            if (gamepad1.dpad_up) {
                wristPosition += wristAdjustment;
            } else if (gamepad1.dpad_down) {
                wristPosition -= wristAdjustment;
            }

            // Ensure wristPosition stays within 0 to 1
            wristPosition = Math.max(0, Math.min(1, wristPosition));

            // Set the wrist servo to the updated position
            wristServo.setPosition(wristPosition);

            // Send telemetry data to the driver station
            telemetry.addData("Wrist Position", wristPosition);
            telemetry.update();
        }

        // Method to control the claw position
        public void controlClaw (Gamepad gamepad1, Telemetry telemetry){
            double clawAdjustment = 0.01; // Tune this for smooth movement

            // Adjust claw position based on gamepad input
            if (gamepad1.a) {  // For example, "A" button opens the claw
                clawPosition += clawAdjustment;
            } else if (gamepad1.b) {  // "B" button closes the claw
                clawPosition -= clawAdjustment;
            }

            // Ensure clawPosition stays within 0 to 1
            clawPosition = Math.max(0, Math.min(1, clawPosition));

            // Set the claw servo to the updated position
            clawServo.setPosition(clawPosition);

            // Send telemetry data to the driver station
            telemetry.addData("Claw Position", clawPosition);
            telemetry.update();
        }
    }