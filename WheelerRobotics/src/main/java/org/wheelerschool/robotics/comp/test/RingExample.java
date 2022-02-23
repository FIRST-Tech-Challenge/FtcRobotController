package org.wheelerschool.robotics.comp.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.wheelerschool.robotics.old.lib.CompBot;
import org.wheelerschool.robotics.comp.auto.BotNav;

@Autonomous
public class RingExample extends OpMode {
    CompBot bot;
    BotNav nav;

    @Override
    public void init() {
        bot = new CompBot(hardwareMap);
        nav = new BotNav(bot);
    }

    @Override
    public void start() {
        nav.activate();
    }

    @Override
    public void loop() {
        telemetry.addData("rings", nav.botVis.ringDetect());
    }
}
