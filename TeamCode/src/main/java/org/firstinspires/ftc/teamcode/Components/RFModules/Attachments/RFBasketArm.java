package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class RFBasketArm extends RFServo {

    private RFServo basketArmServo;

    LinearOpMode op;
    public RFBasketArm(double range, Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        super(range, direction, deviceName, opMode);

        basketArmServo = new RFServo(range, direction, deviceName, opMode);

        op = opMode;
    }

    public void FlipBasketArmToPosition (double torget) {
//        updateTurretPositions(); add later when rf3dturretslides is made
        basketArmServo.setPosition(torget);
    }
}