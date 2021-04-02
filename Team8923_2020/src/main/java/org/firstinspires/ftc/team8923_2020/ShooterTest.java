package org.firstinspires.ftc.team8923_2020;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Shooter Test")

public class ShooterTest extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        //initHardware();
        DcMotor motorShooter = null;
        motorShooter = hardwareMap.dcMotor.get("motorShooter");
        //motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double shootPower = 0.8;

        //motorLift.setTargetPosition(0);
        waitForStart();

        while (opModeIsActive())
        {
            motorShooter.setPower(gamepad1.left_stick_y);
            //if(gamepad1.dpad_right) shootPower += 0.1;
            //if(gamepad1.dpad_left)  shootPower -= 0.1;
            //if(gamepad1.right_trigger > 0.3)  motorShooter.setPower(shootPower);
            //else motorShooter.setPower(0.0);
            //telemetry.addData("Shooter Power", shootPower);
            //telemetry.update();
        }
    }
}
