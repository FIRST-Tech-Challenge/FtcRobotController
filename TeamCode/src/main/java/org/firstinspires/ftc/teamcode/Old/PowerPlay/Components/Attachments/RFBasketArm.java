package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments;

import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.checker;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware.Turret.servoPos;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.isBlue;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.startAngle;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class RFBasketArm extends RFServo {

    private RFServo basketArmServo;
    public RFBasketArm(Servo.Direction direction, String deviceName, double limit) {
        super(deviceName, direction , limit);

        basketArmServo = new RFServo(deviceName, direction , limit);
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