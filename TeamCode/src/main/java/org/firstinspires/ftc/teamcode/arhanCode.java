package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name= "Arhan Cool")
public class arhanCode extends OpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private static double powerInput = 1;


    @Override
    public void loop() {

    }

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        backRight = hardwareMap.get(DcMotor.class, "motorRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void goRight() {
        frontRight.setPower(powerInput);
        frontLeft.setPower(-1 * powerInput);
        backRight.setPower(powerInput);
        backLeft.setPower(-1 * powerInput);
    }

    public void goLeft() {
        frontRight.setPower(-1 * powerInput);
        frontLeft.setPower(-1 * powerInput);
        backRight.setPower(-1 * powerInput);
        backLeft.setPower(-1 * powerInput);
    }

    public void goDiagonalOne() {
        frontRight.setPower(-1 * powerInput);
        backLeft.setPower(-1 * powerInput);
        frontLeft.setPower(0.5 * powerInput);
        backRight.setPower(0.5 * powerInput);
    }



}

