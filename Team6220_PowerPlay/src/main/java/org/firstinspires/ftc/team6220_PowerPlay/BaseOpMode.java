package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

abstract public class BaseOpMode extends LinearOpMode {

    //Declared drive motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    public void initHardware() {
        //init drive motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        //reset encoder
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
    }

    public void driveRobot(double x, double y, double t)  {




        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        //declare Constants
        double turnCorrection = 0.3;
        double offsetComp = startDirection - heading

        double speedFL = (-y+x+t);
        double speedFR = (-y-x-t);
        double speedBL = (-y-x+t);
        double speedBR = (-y+x-t);

        motorFL.setPower(speedFL);
        motorFR.setPower(speedFR);
        motorBL.setPower(speedBL);
        motorBR.setPower(speedBR);


    }

}
