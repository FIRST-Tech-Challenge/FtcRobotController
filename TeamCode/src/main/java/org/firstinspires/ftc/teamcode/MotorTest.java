package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorTest")
public class MotorTest extends OpMode {
    private DcMotor motor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.dcMotor.get("fw");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }

        telemetry.addData("Motor Ticks", ((DcMotorEx)(motor)).getVelocity());
        telemetry.addData("Motor Power", motor.getPower());
    }

}
