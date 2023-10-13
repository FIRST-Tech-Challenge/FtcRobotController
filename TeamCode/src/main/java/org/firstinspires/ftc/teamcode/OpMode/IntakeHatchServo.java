package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.Servo;

public class IntakeHatchServo {

    static Servo hatch;
    static boolean isClosed = true;

    public static void hatchServoInit(Servo servo) {
        hatch = servo;
    }

    public static void hinge(boolean dpadDown) {
        if(dpadDown) {
            if(isClosed) {
                hatch.setPosition(0.25);
                isClosed = !isClosed;
            } else {
                hatch.setPosition(0);
                isClosed = !isClosed;
            }
        }
    }
}
