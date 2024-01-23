package org.firstinspires.ftc.teamcode.Autonomus.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomus.methods.AutoMethods;

@Autonomous(name="AutoTest", group="AutoTest")
//@Disabled
public class AutoTest extends LinearOpMode {

    public AutoMethods bot = new AutoMethods();

    @Override
    public void runOpMode() throws InterruptedException {

        bot.initC(this);

        // Захватываем предзагруженный конус
        bot.Hook.setPosition(0.74);

        waitForStart();

        bot.start();

        bot.vlevo(this, 0.4, 70, 4, 0);

        bot.writeAngle();

    }
}