package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FlyWheelTest")
public class FlyWheelTest extends OpMode {
    private Motor motor;
    private double speedModifier = 0;
    private double direction = 1;
    private boolean pressedA = false;
    private boolean pressedB = false;

    @Override
    public void init() {
        motor = new Motor(hardwareMap, "m");
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(0.05, 0, 0);
    }

    @Override
    public void loop() {

        if(gamepad1.a) {
            pressedA = true;
        } else if(pressedA) {
            pressedA = false;
            direction *= -1;
        }

        if(gamepad1.b) {
            pressedB = true;
        } else if(pressedB) {
            if(speedModifier != 0) {
                speedModifier = 0;
            } else {
                speedModifier = 1;
            }
            pressedB = false;
        }

        double speed = direction * speedModifier * 0.7;

        motor.set(speed);

        telemetry.addData("Motor Speed", motor.get());
        telemetry.addData("Motor Position", motor.getCurrentPosition());


    }
}