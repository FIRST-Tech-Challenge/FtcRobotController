package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFCRServo;

public class RFCarousel extends RFCRServo {

    private RFCRServo carouselCRServo;

    LinearOpMode op;
    public RFCarousel(Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        super(direction, deviceName, opMode);

        carouselCRServo = new RFCRServo(direction, deviceName, opMode);

        op = opMode;
    }

    public  void spinCarouselAutonomousBlue (){
        carouselCRServo.setPower(-0.6);
        op.sleep(4000);
        carouselCRServo.setPower(0);

    }

    public  void spinCarouselAutonomousRed (){
        carouselCRServo.setPower(0.6);
        op.sleep(4000);
        carouselCRServo.setPower(0);
    }


}