package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class FlightSchoolMotorDemo extends OpMode {
    DcMotor motor = null;

    @Override
    public void init() {
        motor = this.hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


    @Override
    public void loop() {
        if(gamepad1.left_trigger > .5)
            motor.setPower(-1);
        else if(gamepad1.right_trigger > .5)
            motor.setPower(1);
        else
            motor.setPower(0);
    }
}
