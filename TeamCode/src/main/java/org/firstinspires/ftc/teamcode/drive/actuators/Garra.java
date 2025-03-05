package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.camera.SensorLimelight3A;

public class Garra extends OpMode {
    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;

    public void init(){
        rotate = hardwareMap.get(Servo.class,"rotate");
        garra = hardwareMap.get(Servo.class,"garra");
        pleft = hardwareMap.get(Servo.class,"pleft");
        pright = hardwareMap.get(Servo.class,"pright");
    }
    public void loop() {

        SensorLimelight3A LimeLight = new SensorLimelight3A();

        if (gamepad1.a) {
            garra.setPosition(0.5);
        }
        if (gamepad1.b) {
            garra.setPosition(1);
        }
        if (gamepad1.x){
            pleft.setPosition(1);
            pright.setPosition(0);
        }
        if (gamepad1.y){
            pleft.setPosition(0.5);
            pright.setPosition(0.5);
        }
        rotate.setPosition(SensorLimelight3A.position(SensorLimelight3A.limelight.getLatestResult().getTyNC()));
    }
}
