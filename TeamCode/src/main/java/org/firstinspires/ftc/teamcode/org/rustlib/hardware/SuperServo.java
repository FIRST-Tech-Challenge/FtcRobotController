package org.firstinspires.ftc.teamcode.org.rustlib.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class SuperServo {
    private final Servo servo;
    private final int polarity;

    public SuperServo(Servo servo, boolean reversed) {
        this.servo = servo;
        polarity = reversed ? -1 : 1;
    }
}
