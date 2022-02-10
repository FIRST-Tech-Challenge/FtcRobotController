package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CappingArm {

    private final Servo cappingServo;
    private final double upPosition = 0.75;
    private final double downPosition = 0.28;
    private final double initPosition = 0.92;
    private final double cappingPosition = 0.54;

    public CappingArm(HardwareMap hardwareMap, String servoName) {
        cappingServo = hardwareMap.servo.get(servoName);
        cappingServo.setPosition(initPosition);
    }

    public void setUpPosition() {
        cappingServo.setPosition(upPosition);
    }

    public void setDownPosition() {
        cappingServo.setPosition(downPosition);
    }

    public void setInitPosition() {
        cappingServo.setPosition(initPosition);
    }

    public void setToCappingPosition() {
        cappingServo.setPosition(cappingPosition);
    }
}
