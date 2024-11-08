package org.firstinspires.ftc.teamcode.functions;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BadServoFunctions {
        public Servo wristServo;
        public Servo wristServo2;
        public Servo clawServo;
        // Variables to store positions for wrist and claw

        double wristPosition = 0.5; // Starting position for wrist 1
        double wristPosition2 = 0.5; // Starting position for wrist 2
        double clawPosition = 0.5; // Starting position for claw

    public BadServoFunctions(HardwareMap hardwareMap){
        wristServo=hardwareMap.get(Servo.class,"wrist_1");
        wristServo2=hardwareMap.get(Servo.class,"wrist_2");
        clawServo=hardwareMap.get(Servo.class,"claw");
    }
        // Method to control the wrist position
        public void controlWrist (Gamepad gamepad1, Telemetry telemetry){
            // good wrist
            if (gamepad1.a) {
                wristServo.setPosition(0.5);
            }

            // Minimum position used for testing purposes
            if (gamepad1.b) {
                wristServo.setPosition(0);
            }
            if (gamepad1.x){
                wristServo2.setPosition(0.5);
            }
            if (gamepad1.y){
                wristServo2.setPosition(0);
            }
            // Get position of the wrist and display on driver station for feedback
            double wristPosition = wristServo.getPosition();
            double wristPosition2 = wristServo2.getPosition();
            telemetry.addData("Wrist Position 1", wristPosition);
            telemetry.addData("Wrist Position 2",wristPosition2);
        }


    // Method to control the claw position
        public void controlClaw (Gamepad gamepad1, Telemetry telemetry){
           if(gamepad1.right_bumper){
               clawServo.setPosition(0);
           }

           if (gamepad1.left_bumper){
               clawServo.setPosition(1);
           }
            // Send telemetry data to the driver station
            double clawPosition = clawServo.getPosition();
            telemetry.addData("Claw Position", clawPosition);
            telemetry.update();
        }
    }