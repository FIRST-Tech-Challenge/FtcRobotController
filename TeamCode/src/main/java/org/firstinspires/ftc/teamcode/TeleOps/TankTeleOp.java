package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TankTele")
public class TankTeleOp extends LinearOpMode {

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;



    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = (DcMotorEx) hardwareMap.get("FL");
        backLeftMotor = (DcMotorEx) hardwareMap.get("BL");
        frontRightMotor = (DcMotorEx) hardwareMap.get("FR");
        backRightMotor = (DcMotorEx) hardwareMap.get("BR");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()){
            drive();
        }
    }

    public void drive(){
        double leftX = gamepad1.left_stick_x;
        double leftY = -gamepad1.left_stick_y;
        double turnVal = gamepad1.right_stick_x;
        double rPower = leftX;
        double lPower = leftX;

        if (leftY > 0){
            lPower -= leftY;
        }
        else if(leftY < 0){
            rPower -= leftY;
        }

        frontRightMotor.setPower(rPower);
        frontLeftMotor.setPower(lPower);
        backRightMotor.setPower(rPower);
        backLeftMotor.setPower(lPower);
    }
}
