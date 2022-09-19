package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.Old.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Old.Components.Hardware.Turret.areTeleop;
import static org.firstinspires.ftc.teamcode.Old.Robots.BlackoutRobot.checker;
import static org.firstinspires.ftc.teamcode.Old.Components.Hardware.Turret.extendPosition;
import static org.firstinspires.ftc.teamcode.Old.Components.Hardware.Turret.rotatePosition;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class RFBasket {

    private RFServo basketServo;

    public RFBasket(Servo.Direction direction, String deviceName) {

        basketServo = new RFServo(direction, deviceName);
    }

    public void FlipBasket () {
//        updateTurretPositions(); add when 3d turret slides done

        if(checker.getState(BASKET_ARM_REST)&&areTeleop) {
            basketServo.setPosition(0.92);
        }
        else{
            basketServo.setPosition(0.18);
            if(extendPosition>400&&abs(rotatePosition)>100){
//                SavePosition(0); add when 3d turret slides done
            }
        }
    }

    public void FlipBasketToPosition (double torget) {
        basketServo.setPosition(torget);
    }

}