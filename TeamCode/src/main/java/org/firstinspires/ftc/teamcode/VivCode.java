package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class VivCode extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()){
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
            fLeft.getCurrentPosition();

            double ticksPerInch = (537.7 / 1.4) / 11.87373601322835;
            if (fLeft.getCurrentPosition() >= ticksPerInch * 12){
                fLeft.setPower(0);
                bLeft.setPower(0);
                fRight.setPower(0);
                bRight.setPower(0);
            } else if (ticksPerInch * 12 > fLeft.getCurrentPosition()){
                fLeft.setPower(0.25);
                bLeft.setPower(0.25);
                fRight.setPower(0.25);
                bRight.setPower(0.25);
            }
        }
    }
}
