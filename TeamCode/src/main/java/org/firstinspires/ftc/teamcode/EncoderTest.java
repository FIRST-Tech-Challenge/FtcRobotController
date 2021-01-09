package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends OpMode {
    private Motor motor;
    private double speedModifier = 1;

    @Override
    public void init() {
        motor = new Motor(hardwareMap, "m");
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(0.05, 0, 0);
    }

    @Override
    public void loop() {

        double speed = 0.25;

        motor.set(speed);

        telemetry.addData("Motor Speed", motor.get());
        telemetry.addData("Motor Position", motor.getCurrentPosition());


    }
}