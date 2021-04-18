package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2020.Constants;
import org.firstinspires.ftc.team6220_2020.MasterTeleOp;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.Button;

@TeleOp(name = "Motor", group = "TeleOp")
public class MotorTest extends MasterTeleOp {

    @Override
    public void runOpMode()
    {
        Initialize();
        waitForStart();
        while(opModeIsActive()){
            motorLauncher.setPower(gamepad1.left_stick_y);

            telemetry.addData("Power", motorLauncher.getPower());
            telemetry.addData("RPM", getMotorTicksPerMinute(motorLauncher, 100) / Constants.AM_37_TICKS_PER_ROTATION);
            telemetry.update();
        }



        //motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
        //while (opModeIsActive()){
//
        //    driveLauncher(gamepad1.left_stick_y);
//
        //    driver1.update();
        //    if(driver1.isButtonPressed(Button.A)){
        //        motorFrontLeft.setPower(.5);
        //    } else if(driver1.isButtonPressed(Button.B)){
        //        motorFrontRight.setPower(.5);
        //    } else if(driver1.isButtonPressed(Button.X)){
        //        motorBackLeft.setPower(.5);
        //    } else if(driver1.isButtonPressed(Button.Y)){
        //        motorBackRight.setPower(.5);
        //    } else{
        //        motorFrontLeft.setPower(0.0);
        //        motorFrontRight.setPower(0.0);
        //        motorBackLeft.setPower(0.0);
        //        motorBackRight.setPower(0.0);
        //    }
//
        //    telemetry.addData("FLsp", getMotorTicksPerMinute(motorFrontLeft, 100) / Constants.AM_20_TICKS_PER_ROTATION);
        //    telemetry.addData("FRsp", getMotorTicksPerMinute(motorFrontRight, 100) / Constants.AM_20_TICKS_PER_ROTATION);
        //    telemetry.addData("BLsp", getMotorTicksPerMinute(motorBackLeft, 100) / Constants.AM_20_TICKS_PER_ROTATION);
        //    telemetry.addData("BRsp", getMotorTicksPerMinute(motorBackRight, 100) / Constants.AM_20_TICKS_PER_ROTATION);
        //   telemetry.addData("FL", motorFrontLeft.getMode());
        //   telemetry.addData("FR", motorFrontRight.getMode());
        //   telemetry.addData("BL", motorBackLeft.getMode());
    }
}

