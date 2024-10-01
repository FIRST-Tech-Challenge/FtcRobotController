package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "intakeTest Joshua")
public class intakeTest extends OpMode
{
    public DcMotor intake;
    public double input;
    public void init()
    {
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set direction
    }

    @Override
    public void loop()
    {
        this.input = gamepad1.right_stick_y;

        double power = 1.0;

        if (input > 0.3) {
            intake.setPower(power);
        }
        else if (input < -0.3)
        {
            intake.setPower(-power);
        }
        else
            intake.setPower(0);
    }
}

