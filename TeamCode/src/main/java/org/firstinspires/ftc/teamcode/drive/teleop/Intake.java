package org.firstinspires.ftc.teamcode.drive.teleop;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class Intake extends OpMode {
    Servo linkageRight;
    Servo linkageLeft;
    Servo pulsoRight;
    Servo pulsoLeft;
    Servo garra;
    Servo rotate;

    public void init(){
        linkageLeft = hardwareMap.get(Servo.class, "lleft");
        linkageRight = hardwareMap.get(Servo.class, "lright");
        pulsoLeft = hardwareMap.get(Servo.class, "pleft");
        pulsoRight = hardwareMap.get(Servo.class, "pright");
        garra = hardwareMap.get(Servo.class, "garra");
        rotate = hardwareMap.get(Servo.class, "rotate");
    }
    public void loop(){
        if (gamepad1.a){
            pulsoRight.setPosition(0.1); // braco volta
            pulsoLeft.setPosition(0.75);
        }
        if (gamepad1.b){
            pulsoRight.setPosition(0.75); // braco coleta
            pulsoLeft.setPosition(0.1);
        }
        if (gamepad1.dpad_down){
            garra.setPosition(1);
            sleep(200);
            pulsoRight.setPosition(0.9);
            pulsoLeft.setPosition(0);
            linkageRight.setPosition(1);
            linkageLeft.setPosition(0);
        }
        if (gamepad1.dpad_up){
            linkageLeft.setPosition(0.35);
            linkageRight.setPosition(0.6);
            sleep(100);
            pulsoRight.setPosition(0);
            pulsoLeft.setPosition(0.85);
            garra.setPosition(0.4);
        }
        if (gamepad1.x){
            garra.setPosition(1); // garra pega
        }
        if(gamepad1.y){
            garra.setPosition(0.4); // garra solta
        }
        if(gamepad1.right_bumper){
            rotate.setPosition(1); // vira pro lado
        }
        if (gamepad1.left_bumper){
            rotate.setPosition(0.65); // meio
        }
    }
    public void bracoColeta() {
        pulsoRight.setPosition(0.1); // braco volta
        pulsoLeft.setPosition(0.75);
    }
    public void bracoVolta() {
        pulsoRight.setPosition(0.75); // braco coleta
        pulsoLeft.setPosition(0.1);
    }
}