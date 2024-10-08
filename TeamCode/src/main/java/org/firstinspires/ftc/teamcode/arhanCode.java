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
    private DcMotor intake;




    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {

        double y = gamepad1.right_stick_y;

        if(y > 0.3)
            moveForward();
        else if(y < -0.3)
            moveBackward();

        double x = gamepad1.right_stick_x;

        if(x > 0.3) {
            goRight();
        } else if(x < -0.3) {
            goLeft();
        }


        /*double power = gamepad1.right_stick_y; //as you move the joy-con R thing it records how much u move
        double x = gamepad1.left_stick_x;
        if (power > 0.3) //to avoid dying i said if the amount u moved it is above 0.3 it will move as much as u move it
            powerInput = power;
        else if (power < -0.3)
            powerInput = power;
        else
            powerInput = 0;*/
    }

    public void moveBackward() {
        frontRight.setPower(-1 * powerInput);
        frontLeft.setPower(-1 * (powerInput));
        backRight.setPower(-1 * powerInput);
        backLeft.setPower(-1 * powerInput);
    }

    public void moveForward() {
        frontRight.setPower(powerInput);
        frontLeft.setPower(powerInput);
        backRight.setPower(powerInput);
        backLeft.setPower(powerInput);
    }

    public void goRight() {
        frontRight.setPower(-1 * powerInput);
        frontLeft.setPower( powerInput);
        backRight.setPower( powerInput);
        backLeft.setPower(-1 * powerInput);
    }

    public void goLeft() {
        frontRight.setPower(powerInput);
        frontLeft.setPower(-1 * powerInput);
        backRight.setPower(-1 * powerInput);
        backLeft.setPower(powerInput);
    }

    public void goLeftDiagonal() {
        frontRight.setPower(powerInput);
        backRight.setPower(0.5 * powerInput);
        frontLeft.setPower(0.5 * powerInput);
        backLeft.setPower(powerInput);
    }

    public void goRightDiagonal() {
        frontRight.setPower(0.5 * powerInput);
        backRight.setPower(0.5 * powerInput);
        frontLeft.setPower(0.5 * powerInput);
        backLeft.setPower(powerInput);
    }




}

