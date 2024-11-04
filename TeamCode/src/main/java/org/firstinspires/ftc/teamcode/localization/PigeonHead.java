package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.OpModeUtilities;

public class PigeonHead {
    OpModeUtilities opModeUtilities;
    private final Servo servo;

    public PigeonHead(Servo servo) {
        this.servo = servo;
    }

    public double positionUpdate(double headingChange) {
        return(-headingChange);
    }

    public double getPigeonHeadPos(Servo servo) {
        return(servo.getPosition());
    }

    public void setPigeonHeadPos(Servo servo, double position) {
        servo.setPosition(position);
    }
}
