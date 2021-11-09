package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_CapArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo gripServo = hardwareMap.get(Servo.class,"gripServo");
        Servo capArm = hardwareMap.get(Servo.class,"capArm");
        boolean dependent = true;
        Servo trapdoor = hardwareMap.get(Servo.class,"trapdoor");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                trapdoor.setPosition(0.2);
            }
            if (gamepad2.dpad_down) {
                trapdoor.setPosition(0.77);
            }
            if (gamepad2.dpad_left) {
                trapdoor.setPosition(0.5);
            }
            if (gamepad2.dpad_right) {
                trapdoor.setPosition(0.0);
            }
        }
    }


}