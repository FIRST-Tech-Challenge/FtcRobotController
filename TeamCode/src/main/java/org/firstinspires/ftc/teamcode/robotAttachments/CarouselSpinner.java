package org.firstinspires.ftc.teamcode.robotAttachments;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinner {
    private CRServo spinnerServo;
    private static final double servoPower = -1;
    private static long duckSleepTime = 1750;

    public CarouselSpinner(HardwareMap hardwareMap, String deviceName){
        spinnerServo = hardwareMap.crservo.get(deviceName);
    }

    public void spinOffRedDuck() throws InterruptedException {
        spinnerServo.setPower(-servoPower);
        Thread.sleep(duckSleepTime);
        spinnerServo.setPower(0);

    }
    public void spinOffBlueDuck() throws InterruptedException {
        spinnerServo.setPower(servoPower);
        Thread.sleep(duckSleepTime);
        spinnerServo.setPower(0);

    }

    public void spinForward(){
        spinnerServo.setPower(servoPower);
    }
    public void spinBackward(){
        spinnerServo.setPower(-servoPower);
    }

    public void stop(){
        spinnerServo.setPower(0);
    }
}
