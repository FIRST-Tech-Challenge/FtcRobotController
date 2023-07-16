package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class RFBasket extends RFServo{

    private RFServo basketServo;

    public RFBasket(Servo.Direction direction, String deviceName, double limit) {
        super(deviceName, direction , limit);

        basketServo = new RFServo(deviceName, direction , limit);
    }

    public RFBasket(String deviceName, double limit) {
        super(deviceName, limit);

        basketServo = new RFServo(deviceName, limit);
    }

//    public void FlipBasket () {
////        updateTurretPositions(); add when 3d turret slides done
//
//        if(checker.getState(BASKET_ARM_REST)&&areTeleop) {
//            basketServo.setPosition(0.92);
//        }
//        else{
//            basketServo.setPosition(0.18);
//            if(extendPosition>400&&abs(rotatePosition)>100){
////                SavePosition(0); add when 3d turret slides done
//            }
//        }
//    }

}