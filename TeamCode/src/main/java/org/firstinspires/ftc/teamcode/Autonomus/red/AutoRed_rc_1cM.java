package org.firstinspires.ftc.teamcode.Autonomus.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomus.methods.AutoMethods;

@Autonomous(name="AutoRed_rc_1cM", group="AutoRed")
//@Disabled
public class AutoRed_rc_1cM extends LinearOpMode {

    public AutoMethods bot = new AutoMethods();

    @Override
    public void runOpMode() throws InterruptedException {

        bot.initC(this);
        bot.camStart(this);

        // Захватываем предзагруженный конус
        bot.Hook.setPosition(0.75);

        waitForStart();

        bot.start();
        bot.getPos();

        int position = 2;

        if (bot.baza == 1) {
            position = 1;
        }
        if (bot.baza == 2) {
            ;
        }
        if (bot.baza == 3) {
            position = 3;
        }

        // Подъезжаем к среднему узлу
        bot.dvizh(this, 0.5, -294, 0, 0, 3.5, false);

        // Выравниваемся
            bot.rotate(0, 1);

        // Поднимаем стрелу
        bot.tele(515);
        sleep(550);

        // Приближаемся к узлу
        bot.vpered(this, 0.3, 36, 2, 0);

        // Опускаем стрелу
        bot.tele(300);
        sleep(550);

        // Отпускаем конус
        bot.Hook.setPosition(0.66);
        sleep(550);

        // Поднимаем стрелу
        bot.tele(515);
        sleep(1050);

        // Отъезжаем от узла
        bot.nazad(this, 0.3, 36, 2, 0);

        // Опускаем стрелу
        bot.tele(0);
        sleep(550);

        // Отъезжаем на место для парковки
        bot.dvizh(this, 0.4, -210, 0, 0, 2.7, false);

        //Паркуемся
        if (position == 1) {
            bot.nazad(this, 0.4, 170, 5,0);
        }
        else if (position == 3) {
            bot.vpered(this, 0.4, 170, 5,0);
        }

        bot.writeAngle();

    }
}