package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp
public class DeliveryGrippersTest extends OpMode {
    public Servo grippers;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public double target = 0;
    @Override
    public void init() {
        grippers = hardwareMap.get(Servo.class, "deliveryGrippers");
        grippers.setPosition(0);
    }

    @Override
    public void loop() {
        if (buttonTimer.seconds() > 0.3 && gamepad1.dpad_right){
            target = target + 0.05;
            buttonTimer.reset();
        }
        else if (buttonTimer.seconds() > 0.3 && gamepad1.dpad_left){
            target = target - 0.05;
            buttonTimer.reset();
        }
        grippers.setPosition(target);
        telemetry.addLine(String.valueOf(target));
    }
}
