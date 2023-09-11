package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

@Config ("RI2D2023-OUTTAKE")
public class Outtake {
    private DcMotorEx slide = null;
    private Servo pixelFliper = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    int slidePosition;
    public static int slidePositionMax = 2000;
    int slideSpeed = 20;
    public static int UNFLIPEDPOSITION = 2000;
    public static int FLIPEDPOSITION = 950;
    private boolean fliped = false;
    public Outtake(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void telemetryOutput()
    {
        telemetry.addData("Slide Position \t", slide.getCurrentPosition());
        telemetry.addData("Pixel Fliper Position \t", Utils.servoDenormalize(pixelFliper.getPosition()));

    }
    public void init()
    {
        slide = this.hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        pixelFliper = this.hardwareMap.get(Servo.class, "pixelFlipper");
    }
    public void moveSlide(int power)
    {
        slidePosition += power*slideSpeed;
        if(slidePosition < 0)
            slidePosition = 0;
        if (slidePosition> slidePositionMax) slidePosition=slidePositionMax;
        slide.setTargetPosition(slidePosition);
    }
    public void flip()
    {
        if(fliped) {
            pixelFliper.setPosition(Utils.servoNormalize(UNFLIPEDPOSITION));
            fliped = false;
        }
        else {
            pixelFliper.setPosition(Utils.servoNormalize(FLIPEDPOSITION));
            fliped = true;
        }
    }
}
