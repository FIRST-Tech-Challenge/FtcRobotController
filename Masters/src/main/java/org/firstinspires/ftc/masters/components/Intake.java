package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    Init init;

    Telemetry telemetry;
    Servo slideServo1;
    Servo slideServo2;
    Servo stringServo;
    DcMotor wheelMotor;

    double extensionPosition;

    public Intake(Init init, Telemetry telemetry){

        this.init = init;

        slideServo1 = init.getSlideServo1();
        slideServo2 = init.getSlideServo2();
        stringServo = init.getStringServo();
        wheelMotor = init.getWheelMotor();
        initializeHardware();

    }

    public void initializeHardware() {

        slideServo1.setPosition(ITDCons.slideInit);
        slideServo2.setPosition(ITDCons.slideInit);
        stringServo.setPosition(ITDCons.liftInit);

    }

    public void intakePower(double power){

        wheelMotor.setPower(power);

    }

    public void slides(double position){

        slideServo1.setPosition(position);
        slideServo2.setPosition(position);

    }

    public void retractSlide() {
      //  extensionPosition = Math.max(extensionPosition - 0.001, .35);

        extensionPosition= ITDCons.slideIn;
        slideServo1.setPosition(extensionPosition);
        slideServo2.setPosition(extensionPosition);
    }

    public void extendSlide() {
//        extensionPosition = /*Math.min(extensionPosition + 0.001, .5);
        extensionPosition= ITDCons.slideOut;
        slideServo1.setPosition(extensionPosition);
        slideServo2.setPosition(extensionPosition);
    }

    public void intakeLift(double position){

        stringServo.setPosition(position);

    }

    public double getExtensionPosition(){
        return extensionPosition;
    }

}
