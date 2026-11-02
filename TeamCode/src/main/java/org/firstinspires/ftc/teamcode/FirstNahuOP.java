package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp



public class FirstNahuOP extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    @Override
    public void runOpMode()
    {
        leftMotor = hardwareMap.get(DcMotor.class,"lMotor");
        rightMotor = hardwareMap.get(DcMotor.class,"rMotor");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {

            if(gamepad1.y)
            {
                leftMotor.setPower(-1);
                rightMotor.setPower(1);
                sleep(1000);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            if(gamepad1.a)
            {
                leftMotor.setPower(1);
                rightMotor.setPower(-1);
                sleep(1000);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            if(gamepad1.x)
            {
                leftMotor.setPower(1);
                rightMotor.setPower(1);
                sleep(500);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            if(gamepad1.b)
            {
                leftMotor.setPower(-1);
                rightMotor.setPower(-1);
                sleep(500);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            telemetry.addData("Status","Running");
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
