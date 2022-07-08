package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.Components.Turret.checker;
import static org.firstinspires.ftc.teamcode.Components.Turret.extendPosition;
import static org.firstinspires.ftc.teamcode.Components.Turret.rotatePosition;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.StateMachine;

public class RFBasket extends RFServo {

    private RFServo basketServo;

    LinearOpMode op;
    public RFBasket(double range, Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        super(range, direction, deviceName, opMode);

        basketServo = new RFServo(range, direction, deviceName, opMode);

        op = opMode;
    }

//    public void FlipBasket (int up) {
////        updateTurretPositions(); add when 3d turret slides done
//
//        if(checker.getState(StateMachine.States.BASKET_ARM_REST)&&isTeleop) {
//            basketServo.setPosition(0.92);
//        }
//        else{
//            basketServo.setPosition(0.18);
//            if(extendPosition>400&&abs(rotatePosition)>100){
//                SavePosition(0);
//            }
//        }
//    }

}