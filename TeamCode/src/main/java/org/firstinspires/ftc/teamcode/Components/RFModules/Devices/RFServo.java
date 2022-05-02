package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class RFServo {
    //all servo regular stuff

    private Servo RFServo;

    LinearOpMode op;

    public RFServo (double range, Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        op = opMode;
        RFServo = opMode.hardwareMap.get(Servo.class, deviceName);
    }

    void setPosition(double position) {
        RFServo.setPosition(position);
    }

    double getPosition() {
        return RFServo.getPosition();
    }

    void setDirection (Servo.Direction direction) {
        RFServo.setDirection(direction);
    }
}
