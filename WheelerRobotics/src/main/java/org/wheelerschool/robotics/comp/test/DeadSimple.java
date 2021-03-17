package org.wheelerschool.robotics.comp.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.wheelerschool.robotics.comp.CompBot;

@Autonomous
public class DeadSimple extends OpMode {
    CompBot bot;

    @Override
    public void init() {
        bot = new CompBot(hardwareMap);
    }

    @Override
    public void loop() {
        bot.setDrive(0, 0, 0.25f);
    }
}
