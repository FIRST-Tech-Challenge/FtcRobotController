package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp( name = "Move Fwd")
public class MoveFwd extends LinearOpMode {

    DcMotor leftMoter = null;
    DcMotor rightMoter = null;
    DcMotor backLeft = null;

    DcMotor backRight = null;

    @Override
    public void runOpMode() throws InterruptedException {
         leftMoter = hardwareMap.get(DcMotor.class,"frontleft");
         rightMoter = hardwareMap.get(DcMotor.class, "frontright");
         backLeft = hardwareMap.get(DcMotor.class, "backleft");
         backRight = hardwareMap.get(DcMotor.class, "backright");

        leftMoter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMoter.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Instr", "Click Start to mmove FWD");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            move(0.5,3000);
            sleep(1000);
            move(-0.25,3000);
        }


    }


    private void move(double power , int sleepTime) {
        leftMoter.setPower(power);
        rightMoter.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        sleep(sleepTime);
        leftMoter.setPower(0);
        rightMoter.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }



}
