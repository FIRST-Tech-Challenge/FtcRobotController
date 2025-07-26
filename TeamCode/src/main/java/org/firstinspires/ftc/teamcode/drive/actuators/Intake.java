package org.firstinspires.ftc.teamcode.drive.actuators;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
public class Intake extends OpMode {
        Servo lright;
        Servo lleft;
        Servo rotate;
        Servo garra;
        Servo pleft;
        Servo pright;

        public void init(){
            lright = hardwareMap.get(Servo.class, "lright");
            lleft = hardwareMap.get(Servo.class, "lleft");
            rotate = hardwareMap.get(Servo.class, "rotate");
            garra = hardwareMap.get(Servo.class, "garra");
            pleft = hardwareMap.get(Servo.class, "pleft");
            pright = hardwareMap.get(Servo.class, "pright");
        }

        public void loop(){
            extendsIntake();
        }

        public void extendsIntake() {
            lright.setPosition(0.6);
            lleft.setPosition(0.7);
            sleep(200);
            garra.setPosition(0.3);
            pleft.setPosition(0);
            pright.setPosition(1);
        }
}
