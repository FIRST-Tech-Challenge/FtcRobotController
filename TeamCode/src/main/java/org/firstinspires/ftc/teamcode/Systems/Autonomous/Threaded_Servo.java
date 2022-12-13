package org.firstinspires.ftc.teamcode.Systems.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

public class Threaded_Servo extends Thread {

    Servo servo;
    double target_position;

    public Threaded_Servo(RobotHardware r, String servo_name) {
        servo = r.map.get(Servo.class, servo_name);
    }

    public void set_position(double p) {
        target_position = p;
    }

    public boolean should_be_running = true;

    public void run() {
        while (should_be_running) {
            servo.setPosition(target_position);
        }
    }

}