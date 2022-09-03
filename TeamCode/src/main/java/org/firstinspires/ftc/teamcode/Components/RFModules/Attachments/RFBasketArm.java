package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.BlackoutRobot.checker;
import static org.firstinspires.ftc.teamcode.Components.Turret.servoPos;
import static org.firstinspires.ftc.teamcode.BlackoutRobot.isBlue;
import static org.firstinspires.ftc.teamcode.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.BlackoutRobot.startAngle;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.StateMachine;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFBasketArm extends RFServo {

    private RFServo basketArmServo;

    LinearOpMode op;
    public RFBasketArm(Servo.Direction direction, String deviceName) {
        super(direction, deviceName);

        basketArmServo = new RFServo(direction, deviceName);
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