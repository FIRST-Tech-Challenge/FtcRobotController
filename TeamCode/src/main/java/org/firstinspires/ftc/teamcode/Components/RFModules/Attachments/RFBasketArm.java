package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketBasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Components.Turret.checker;
import static org.firstinspires.ftc.teamcode.Components.Turret.servoPos;
import static org.firstinspires.ftc.teamcode.Robot.isBlue;
import static org.firstinspires.ftc.teamcode.Robot.startAngle;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.StateMachine;

public class RFBasketArm extends RFServo {

    private RFServo basketArmServo;

    LinearOpMode op;
    public RFBasketArm(double range, Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        super(range, direction, deviceName, opMode);

        basketArmServo = new RFServo(range, direction, deviceName, opMode);

        op = opMode;
    }

    public void FlipBasketArmToPosition (double torget) {
//        updateTurretPositions(); add when 3d turret slides done
        basketArmServo.setPosition(torget);
    }

    public void FlipBasketArmLow () {
        if(checker.getState(BASKET_ARM_REST)) {
            basketArmServo.setPosition(0.9);
        }
        else{
            basketArmServo.setPosition(0.00);
        }
        servoPos = !servoPos;
    }

    public void FlipBasketArmHigh () {
//        updateTurretPositions(); add when 3d turret slides done
        if(abs(EncoderChassis.angle)<45- startAngle*isBlue) {
            if (checker.getState(BASKET_ARM_REST)) {
                basketArmServo.setPosition(0.45);
            } else {
                basketArmServo.setPosition(0.01);
            }
        }
        else{
            if (basketArmServo.getPosition()<0.7) {
                basketArmServo.setPosition(0.8);
            } else {
                basketArmServo.setPosition(0.01);
            }
        }
    }
}