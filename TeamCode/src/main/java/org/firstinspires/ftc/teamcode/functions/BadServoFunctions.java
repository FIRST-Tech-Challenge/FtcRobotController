package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BadServoFunctions {
        public Servo wristServo;
        public Servo wristServo2;
        public Servo clawServo;

        public BadServoFunctions(HardwareMap hardwareMap) {
            wristServo = hardwareMap.get(Servo.class,"wrist_1");
            wristServo2 = hardwareMap.get(Servo.class,"wrist_2");
            clawServo = hardwareMap.get(Servo.class,"claw");
        }

        // Method to control the wrist position
        public void controlWrist (Gamepad gamepad2, Telemetry telemetry) {
            // Get position of the wrist so it knows
            double wristPosition = wristServo.getPosition();
            double wristPosition2 = wristServo2.getPosition();

            //set wrist to correct position at beginning of match !BE WARY OF THIS!
            if (wristPosition == 0){
                wristServo.setPosition(0.5);
            }

            if (wristPosition==0.5){
                wristServo2.setPosition(0.5);
            } else if (wristPosition==1) {
                wristServo2.setPosition(0.5);
            }

            // horrible wrist
            if (gamepad2.y) {
                wristServo.setPosition(1);
            } else if (gamepad2.x){
                wristServo.setPosition(0.5);
            }

            //sending telemetry data
            telemetry.addData("Wrist 1 Position", wristPosition);
            telemetry.addData("Wrist 2 Position",wristPosition2);
        }

        // Method to control the claw position
        public void controlClaw (Gamepad gamepad2, Telemetry telemetry){
            clawServo.setPosition(0.5); // Starting position for claw
            double clawPosition = clawServo.getPosition();

           if(gamepad2.right_bumper){
               clawServo.setPosition(0);
           } else if (gamepad2.left_bumper) {
               clawServo.setPosition(-1);
           }

            // Send telemetry data to the driver station

            telemetry.addData("Claw Position", clawPosition);
            telemetry.update();
        }

    }