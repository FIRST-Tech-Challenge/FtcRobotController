package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSubsystem extends SubsystemBase {
    private final CRServoImpl servo;

    public CarouselSubsystem(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(CRServoImpl.class, "carousel");
    }

    public void rotateClockwise() {
        servo.setPower(1);
    }

    public void rotateCounterClockwise() {
        servo.setPower(-1);
    }

    public void stop() {
        servo.setPower(0);
    }
}