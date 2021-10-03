package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp
public class TestWight  extends OpMode {
    private DcMotorEx motor;

    FileWriter out;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "Motor");

        motor.setTargetPosition(0);

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        var pid = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        pid.p = 25; // Default is 10
        pid.i = 0.005; // Default is 0.05/
//        pid.d = 0.01;

        telemetry.addData("pid", () -> motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.update();

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);

        motor.setPower(1);

        telemetry.addData("position", motor::getCurrentPosition);
        telemetry.addData("velocity", motor::getVelocity);
        telemetry.update();
    }

    @SuppressLint({"SdCardPath", "SimpleDateFormat"})
    @Override
    public void start() {
        try {
            out = new FileWriter("/sdcard/FIRST/logs/" + new SimpleDateFormat("yyyy MM dd HH mm ss").format(new Date()) + ".csv");
        } catch (IOException e) {
            out = null;
        }
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        if (gamepad1.a) {
            motor.setTargetPosition(20);
        } else if (gamepad1.b) {
            motor.setTargetPosition(0);
        }
        telemetry.update();
        try {
            if (out != null)
                out.write(String.format("%f,%f,%f\n", getRuntime(), motor.getPower(), motor.getVelocity()));
        } catch (IOException ignored) {}
    }

    @Override
    public void stop() {
        try {
            if (out != null)
                out.close();
        } catch (IOException ignored) {}
    }
}
