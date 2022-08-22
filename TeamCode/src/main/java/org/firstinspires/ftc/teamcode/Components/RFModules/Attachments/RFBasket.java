package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Components.Turret.areTeleop;
import static org.firstinspires.ftc.teamcode.Robot.checker;
import static org.firstinspires.ftc.teamcode.Components.Turret.extendPosition;
import static org.firstinspires.ftc.teamcode.Components.Turret.rotatePosition;
import static org.firstinspires.ftc.teamcode.Robot.logger;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.StateMachine;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFBasket {

    private RFServo basketServo;

    LinearOpMode op;
    public RFBasket(double range, Servo.Direction direction, String deviceName, LinearOpMode opMode) {

        basketServo = new RFServo(range, direction, deviceName, opMode);

        op = opMode;
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

}