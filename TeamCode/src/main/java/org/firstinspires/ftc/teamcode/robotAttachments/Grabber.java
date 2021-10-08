package org.firstinspires.ftc.teamcode.robotAttachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    private DcMotor linearSlide;
    private Servo grabberServo;

    private static final float openPosition  = 0.5f;
    private static final float closePosition = 0.6f;

    private static final int encoderTicksPerVerticalInch = 100;
    private static final float motorDrivePower = 0.5f;

    Grabber(HardwareMap hardwareMap, String servoName, String motorName){
        linearSlide = hardwareMap.dcMotor.get(motorName);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); //This may be broken in the library implementation
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberServo = hardwareMap.servo.get(servoName);
    }

    public void goToHeight(int height){
        linearSlide.setTargetPosition(height*encoderTicksPerVerticalInch);

    }

    public void open(){
        grabberServo.setPosition(openPosition);
    }
    public void close(){
        grabberServo.setPosition(closePosition);
    }
}
