package org.wheelerschool.robotics.comp.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.comp.CompBot;
import org.wheelerschool.robotics.comp.auto.BotNav;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

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
