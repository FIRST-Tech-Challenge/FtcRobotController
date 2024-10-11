package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestServo  extends TestItem {
    private Servo servo;
    private double onValue;
    private double offValue;

    public TestServo(String description, Servo servo, double onValue, double offValue){
        super(description);
        this.servo = servo;
        this.onValue = onValue;
        this.offValue = offValue;
    }

    @Override
    public void run(boolean on, Telemetry telemetry){
        if (on){
            servo.setPosition(onValue);
        } else {
            servo.setPosition(offValue);
        }
    }
}
