package org.firstinspires.ftc.teamcode.gen1_Qualifying;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_CapArmLocations extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo gripServo = hardwareMap.get(Servo.class,"gripServo");
        Servo capArm = hardwareMap.get(Servo.class,"capArm");
        boolean arm = false;
        boolean grip = false;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.x){
                capArm.setPosition(0.0);
            }
            if (gamepad2.y){
                capArm.setPosition(0.25);
            }
            if (gamepad2.a){
                capArm.setPosition(0.77);
            }
            if (gamepad2.b){
                capArm.setPosition(1);

            }

            }
        }
    }
