package org.firstinspires.ftc.teamcode.drive.actuators;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
public class Intake {
        Servo lright;
        Servo lleft;
        Servo rotate;
        Servo garra;
        Servo pleft;
        Servo pright;

    public Intake(HardwareMap hardwareMap) {
        lright = hardwareMap.get(Servo.class, "lright");
        lleft = hardwareMap.get(Servo.class, "lleft");
        rotate = hardwareMap.get(Servo.class, "rotate");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");
    }

        public void extendsIntake() {
            lright.setPosition(0.6);
            lleft.setPosition(0.7);
            sleep(200);
            garra.setPosition(0.3);
            pleft.setPosition(0);
            pright.setPosition(1);
        }
        public void retractsIntake(){
            garra.setPosition(0.6);
            sleep(200);
            rotate.setPosition(0.7);
            pleft.setPosition(0.75);
            pright.setPosition(0.25);
            lright.setPosition(1);
            lleft.setPosition(0.1);
            sleep(500);
            garra.setPosition(0.3);
        }
}
