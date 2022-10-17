package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    private Button ctrlProc;
    private Button ctrlRunForward;
    private Button ctrlRunReverse;
    private CRServoImpl servo;
    private boolean runProc = false;

    public Carousel(HardwareMap hardwareMap, ExtendedGamepad extGamepad2) {
        this.ctrlProc = extGamepad2.b;
        this.ctrlRunForward = extGamepad2.left_trigger_button;
        this.ctrlRunReverse = extGamepad2.right_trigger_button;
        this.servo = hardwareMap.get(CRServoImpl.class, "carousel");
        this.runProc = false;
    }

    private void sleep(int ms) {
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < ms) {}
    }

    public void run() {
        if (ctrlProc.isBumped()) {
//            for (int i = 0; i < 10; i++) {
//                servo.setPower(1);
//                this.sleep(2000);
//                servo.setPower(0);
//                this.sleep(500);
//            }
//            runProc = true;
        } else if (ctrlRunForward.isPressed()) {
            servo.setPower(1);
        } else if (ctrlRunReverse.isPressed()) {
            servo.setPower(-1);
        }  else if (!runProc) {
            servo.setPower(0);
        }
    }
}