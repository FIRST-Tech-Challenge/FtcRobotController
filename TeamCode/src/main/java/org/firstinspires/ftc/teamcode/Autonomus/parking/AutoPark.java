package org.firstinspires.ftc.teamcode.Autonomus.parking;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomus.methods.AutoMethods;

@Autonomous(name="AutoPark", group="AutoPark")
@Disabled
public class AutoPark extends LinearOpMode {

    public AutoMethods bot = new AutoMethods();

    @Override
    public void runOpMode() throws InterruptedException {

        bot.initC(this);
        bot.camStart(this);

        // Захватываем предзагруженный конус
        bot.Hook.setPosition(0.74);

        waitForStart();

        bot.start();
        sleep(1000);
        bot.getPos();

        int position = 2;

        if (bot.baza == 1) {
            position = 1;
        }
        if (bot.baza == 2) {
            position = 2;
        }
        if (bot.baza == 3) {
            position = 3;
        }

        bot.vlevo(this, 0.5, 210, 4, 0);

        // Паркуемся
        if (position == 1) {
            sleep(1000);
            bot.nazad(this, 0.4, 170, 4.5, 0);
        }
        else if (position == 3) {
            sleep(1000);
            bot.vpered(this, 0.4, 170, 4.5, 0);
        }

        bot.writeAngle();

    }
}