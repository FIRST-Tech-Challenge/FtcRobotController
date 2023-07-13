package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFCRServo;

public class RFCarousel extends RFCRServo {

    private RFCRServo carouselCRServo;

    public RFCarousel(Servo.Direction direction, String deviceName) {
        super(direction, deviceName);

        carouselCRServo = new RFCRServo(direction, deviceName);
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