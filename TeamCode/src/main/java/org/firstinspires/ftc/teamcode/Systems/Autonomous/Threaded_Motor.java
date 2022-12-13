package org.firstinspires.ftc.teamcode.Systems.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

public class Threaded_Motor extends Thread {

    DcMotor motor;
    double min_power;
    double max_power;
    double p;
    int target_position;
    public boolean isBusy = false;
    long startTime = 0;
    long delay = 0;
    long currentTime = System.nanoTime();

    public Threaded_Motor(RobotHardware r, String motor_name) {
        motor = r.map.get(DcMotor.class, motor_name);
        min_power = r.min_power[r.dc_motor_names.indexOf(motor_name)];
        max_power = r.max_power[r.dc_motor_names.indexOf(motor_name)];
        p = r.p_weights[r.dc_motor_names.indexOf(motor_name)];
    }

    public boolean should_be_running = true;

    public void set_position(int p) {
        target_position = p;
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void delay(int milliseconds) {
        delay = milliseconds * 1000000L;
        startTime = System.nanoTime();
    }

    public void run() {
        reset();
        while (should_be_running) {
            currentTime = System.nanoTime();
            if (currentTime > startTime + delay) {
                isBusy = (Math.abs(target_position - motor.getCurrentPosition()) < 5);
                motor.setPower(Math.max(-0.5, Math.min(0.5, p *
                        (target_position - motor.getCurrentPosition())
                )));
            } else {
                isBusy = true;
                motor.setPower(0);
            }
        }
    }

}