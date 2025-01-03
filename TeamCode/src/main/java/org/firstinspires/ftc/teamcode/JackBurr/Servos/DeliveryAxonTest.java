package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DeliveryAxonTest extends OpMode {
    public Servo axon;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public double target = 0;
    @Override
    public void init() {
        axon = hardwareMap.get(Servo.class, "deliveryServo");
        axon.setPosition(0);
    }

    @Override
    public void loop() {
        if (buttonTimer.seconds() > 0.3 && gamepad1.dpad_right){
            target = target + 0.01;
            buttonTimer.reset();
        }
        else if (buttonTimer.seconds() > 0.3 && gamepad1.dpad_left){
            target = target - 0.01;
            buttonTimer.reset();
        }
        axon.setPosition(target);
        telemetry.addLine(String.valueOf(target));
    }
}
