package org.firstinspires.ftc.teamcode.League1.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

public class EndgameSystems {
    /**
     * Class for the Carousel and the Capstone mech
     * Electronics Schematic,
     *
     * Capstone Mechanism
     * 2x crServo
     * 1xServo
     *
     * Carousel
     * 1xCRServo
     * 1xEncoder
     */

    private CRServo xCap;
    private Servo yCap;
    private DcMotor capstoneExtension;
    private boolean isBlue;

    public double xCapSpeedDiv = 7;
    private double yCapSpeed = -0.15;




    public EndgameSystems(HardwareMap hardwareMap, boolean isBlue){
        xCap = hardwareMap.crservo.get("xCap");
        yCap = hardwareMap.servo.get("yCap");
        capstoneExtension = hardwareMap.dcMotor.get("WinchEncoder");

        //carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.isBlue = isBlue;
        // zeroCap();

    }

    public void setCapstoneExtensionPower(double pow){
        capstoneExtension.setPower(-pow);
    }

    public void setXCapSpeedDivisor(double div) {
        xCapSpeedDiv = div;
    }

    public void setXCapstoneRotatePower(double pow) {
        xCap.setPower(-pow / xCapSpeedDiv);
    }

    public void setYCapPosition(double pos){
        yCap.setPosition(pos);
    }

    public void zeroCap() {
        yCap.setPosition(0.45);
        // TODO - get the x position to set the x zero pos (not fully necessary, might be nice to have)
    }

    public double map(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


    public double getYCapPosition() {
        return yCap.getPosition();
    }
}