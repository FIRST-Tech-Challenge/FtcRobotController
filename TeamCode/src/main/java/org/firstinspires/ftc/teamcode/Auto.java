package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Auto extends LinearOpMode {
    public void encoderMove(int inches) {
        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.getCurrentPosition();

        double ticksPerInch = (537.7 / 1.4) / 11.87373601322835;

        while(Math.abs(fLeft.getCurrentPosition()) <=ticksPerInch * Math.abs(inches)&&opModeIsActive()){

            if (Math.abs(fLeft.getCurrentPosition()) >=ticksPerInch*Math.abs(inches)){
                fLeft.setPower(0);
                bLeft.setPower(0);
                fRight.setPower(0);
                bRight.setPower(0);
            }
            else {
                if (inches > 0) {
                    fLeft.setPower(0.25);
                    bLeft.setPower(0.25);
                    fRight.setPower(0.25);
                    bRight.setPower(0.25);
                }
                if (inches < 0) {
                    fLeft.setPower(-0.25);
                    bLeft.setPower(-0.25);
                    fRight.setPower(-0.25);
                    bRight.setPower(-0.25);
                }
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException{

        waitForStart();
        encoderMove(12); // moves forward 12 inches
        encoderMove(-12); // moves backward 12 inches
    }

}



