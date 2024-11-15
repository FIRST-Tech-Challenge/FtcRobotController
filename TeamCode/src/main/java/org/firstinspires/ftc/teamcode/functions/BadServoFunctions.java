package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BadServoFunctions {

        public Servo wrist;
        public Servo claw;

        public BadServoFunctions(HardwareMap hardwareMap) {

            wrist = hardwareMap.get(Servo.class,"wrist_1");
            claw = hardwareMap.get(Servo.class,"claw");
        }

        // Method to control the wrist position
        public void controlWrist (Gamepad gamepad2, Telemetry telemetry) {

            // Get position of the wrist so it knows
            double wristPosition = wrist.getPosition();

            //set wrist to correct position at beginning of match !BE WARY OF THIS!
            if (wristPosition == 0){
                wrist.setPosition(0.5);
            }

            // horrible wrist
            if (gamepad2.y) {
                wrist.setPosition(1);
            } else if (gamepad2.x){
                wrist.setPosition(0.5);
            }

            //sending telemetry data
            telemetry.addData("Wrist 1 Position", wristPosition);
        }

        // Method to control the claw position
        public void controlClaw (Gamepad gamepad2, Telemetry telemetry){

           if(gamepad2.a) {
               claw.setPosition(0);
           }else if (gamepad2.b){
               claw.setPosition(0.5);
           }

            // Send telemetry data to the driver station
            double clawPosition = claw.getPosition();
            telemetry.addData("Claw Position", clawPosition);
            telemetry.update();
        }

    }