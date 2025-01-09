package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AxonHoldAtOne extends OpMode {
    public DiffConstantsV1 diffConstantsV1 = new DiffConstantsV1();
    public Servo grippers;
    public Servo left_servo;
    public AnalogInput left_servo_encoder;
    public AnalogInput right_servo_encoder;
    public Servo right_servo;
    public ElapsedTime button_timer = new ElapsedTime();
    public ElapsedTime servoTimer = new ElapsedTime();
    public ElapsedTime elbowTimer = new ElapsedTime();
    @Override
    public void init() {
      left_servo.setPosition(1);
      right_servo.setPosition(1);
    }

    @Override
    public void loop() {
        left_servo.setPosition(1);
        right_servo.setPosition(1);
    }
}
