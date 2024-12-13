package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

public class BucketSubsystem extends SubsystemBase {

    final ServoEx servo;
    static final float MAX_POSITION = 90 + 5; // Was 90 + 25 as of 12/9/2024

    public BucketSubsystem(ServoEx servoEx) {
        this.servo = servoEx;
        this.servo.setRange(0, MAX_POSITION);
    }

    public void moveToNormalPosition(boolean normalPosition) {
        servo.setPosition(normalPosition ? 0.05 : .8f);
    }

    public void dumpBucket() {
        moveToNormalPosition(false);
    }

    public void resetBucket() {
        moveToNormalPosition(true);
    }

}
