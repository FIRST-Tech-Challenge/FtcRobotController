package org.firstinspires.ftc.teamcode.Functions.Auxiliary;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AuxiliaryExempleMain extends OpMode {

    DcMotor leftMotor, rightMotor;
    Servo testServo;
    AuxiliaryTemplate auxiliaryTemplate;
    @Override
    public void init() {
        testServo = hardwareMap.servo.get("TS");
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        auxiliaryTemplate = new AuxiliaryTemplate(this, leftMotor, testServo, rightMotor);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_left){
            auxiliaryTemplate.SwitchAndWait(1);
        }
        else if(gamepad1.dpad_right){
            auxiliaryTemplate.SwitchAndWaitReverse(1);
        }

    }
}
