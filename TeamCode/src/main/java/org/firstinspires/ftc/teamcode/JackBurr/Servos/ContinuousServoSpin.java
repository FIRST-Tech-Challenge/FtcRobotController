package org.firstinspires.ftc.teamcode.JackBurr.Servos;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Continuous Servo Spin")
public class ContinuousServoSpin extends OpMode {

    private Servo servo;
    private double currentPosition = 0;
    private final double MAX_POSITION = 1;
    private final double STEP_SIZE = 0.01;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        currentPosition += STEP_SIZE;

        if (currentPosition > MAX_POSITION) {
            currentPosition -= MAX_POSITION * 2; // Wrap around to the beginning
        }
        servo.setPosition(currentPosition);
        telemetry.addData("Servo Position", currentPosition);
        telemetry.update();

    }
}