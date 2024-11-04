package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.OpModeUtilities;

public class PigeonHead {
    OpModeUtilities opModeUtilities;
    private final Servo servo;

    public PigeonHead(Servo servo, SparkFunOTOS myOtos) {
        this.servo = servo;
    }

    public double positionUpdate(double heading) {
        return(-heading);
    }
}
