package org.firstinspires.ftc.team8923_2020;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Shooter Test")

@Disabled
public class ShooterTest extends MasterTeleOp
{
    @Override
    public void runOpMode()
    {
        //initHardware();
        DcMotor motorWobble = null;
        motorWobble = hardwareMap.dcMotor.get("motorWobble");
        motorWobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWobble.setTargetPosition(motorWobble.getCurrentPosition());
        motorWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad2.left_stick_y > 0.3 || gamepad2.left_stick_y < -0.3) {
                motorWobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorWobble.setPower(-gamepad2.left_stick_y * 0.25);
            } else {
                motorWobble.setTargetPosition(motorWobble.getCurrentPosition());
                motorWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //motorWobble.setPower(Math.max((motorWobble.getTargetPosition() - motorWobble.getCurrentPosition()) * (1 / 75.0), 1.0));
                motorWobble.setPower(1.0);
            }
        }
    }


}
