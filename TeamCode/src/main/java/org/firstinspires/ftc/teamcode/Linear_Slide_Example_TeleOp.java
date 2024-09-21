package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Linear Slide Example Teleop", group="Example")
public class Linear_Slide_Example_TeleOp extends LinearOpMode {

    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {


        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y;

            if (power > 0 && motor.getCurrentPosition() > 4250)
            {
                power = 0;
            } else if (power < 0 && motor.getCurrentPosition() < 0) {
                power = 0;
            }

            telemetry.addData(">", motor.getCurrentPosition());
            telemetry.update();

            motor.setPower(power);

        }
    }
}