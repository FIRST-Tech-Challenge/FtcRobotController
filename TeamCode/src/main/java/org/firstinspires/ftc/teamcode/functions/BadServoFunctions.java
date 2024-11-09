package org.firstinspires.ftc.teamcode.functions;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BadServoFunctions {
        public Servo wristServo;
        public Servo wristServo2;
        public Servo clawServo;


    public BadServoFunctions(HardwareMap hardwareMap){
        wristServo=hardwareMap.get(Servo.class,"wrist_1");
        wristServo2=hardwareMap.get(Servo.class,"wrist_2");
        clawServo=hardwareMap.get(Servo.class,"claw");

        // initialization positions
        wristServo.setPosition(0.5); // Starting position for wrist 1
        wristServo2.setPosition(0.5); // Starting position for wrist 2
        clawServo.setPosition(0.5); // Starting position for claw
    }
        // Method to control the wrist position
        public void controlWrist (Gamepad gamepad1, Telemetry telemetry){
            // Get position of the wrist so it knows
            double wristPosition = wristServo.getPosition();
            double wristPosition2 = wristServo2.getPosition();

            // good wrist
            if (gamepad1.a && wristPosition == 0) {
                wristServo.setPosition(0.5);
            } else if (gamepad1.a && wristPosition == 0.5){
                wristServo.setPosition(0);
            }

            if (gamepad1.b & wristPosition2 ==0) {
                wristServo.setPosition(0.5);
            } else if (gamepad1.b && wristPosition2 == 0) {
                wristServo.setPosition(0);
            }

            //sending telemetry data
            telemetry.addData("Wrist Position 1", wristPosition);
            telemetry.addData("Wrist Position 2",wristPosition2);
        }


    // Method to control the claw position
        public void controlClaw (Gamepad gamepad1, Telemetry telemetry){
           if(gamepad1.right_bumper){
               clawServo.setPosition(0);
           } else if (gamepad1.left_bumper) {
               clawServo.setPosition(1);
           }

            // Send telemetry data to the driver station
            double clawPosition = clawServo.getPosition();
            telemetry.addData("Claw Position", clawPosition);
            telemetry.update();
        }
    }