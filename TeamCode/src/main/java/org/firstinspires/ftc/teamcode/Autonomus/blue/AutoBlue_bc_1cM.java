package org.firstinspires.ftc.teamcode.Autonomus.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomus.methods.AutoMethods;

@Autonomous(name="AutoBlue_bc_1cM", group="AutoBlue")
//@Disabled
public class AutoBlue_bc_1cM extends LinearOpMode {

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
        bot.dvizh(this, 0.5, -294, 0, 0, 2, false);

        sleep(500);
        // Выравниваемся
        bot.rotate(180, 1.5);
        sleep(300);

        // Поднимаем стрелу
        bot.tele(340);
        sleep(550);

        // Приближаемся к узлу
        //bot.dvizh(this, 0.5, -294, 39, 179.9, 2, false);
        bot.nazad(this, 0.5, 37, 2, 180);// !!!

        // Отпускаем конус
        bot.Hook.setPosition(0.66);
        sleep(550);

        // Поднимаем стрелу
        bot.tele(400);
        sleep(550);

        // Отъезжаем от узла
        bot.vpered(this, 0.5, 37, 2, 180); // !!!

        // Опускаем стрелу
        bot.tele(0);
        sleep(550);

        bot.rotate(0, 1.5);

        // Отъезжаем на место для парковки
        bot.dvizh(this, 0.4, -210, 0, 0, 2.7, false);

        //Паркуемся
        if (position == 1) {
            bot.nazad(this, 0.4, 170, 5, 0);
        }
        else if (position == 3) {
            bot.vpered(this, 0.4, 170, 5, 0);
        }

        bot.writeAngle();

    }
}