package org.firstinspires.ftc.robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot_utilities.Vals;

public class Hitter {

    public Servo hitter;

    public Hitter(Servo hitter) {
        this.hitter = hitter;
    }

    public void hit() {
        hitter.setPosition(Vals.hitter_end);
    }

    public void reset() {
        hitter.setPosition(Vals.hitter_start);
    }

    public void hitFullMotion(double delay) {
        ElapsedTime elapsedTime = new ElapsedTime();
        while(elapsedTime.seconds() < delay) this.hit();

        this.reset();

    }

}
