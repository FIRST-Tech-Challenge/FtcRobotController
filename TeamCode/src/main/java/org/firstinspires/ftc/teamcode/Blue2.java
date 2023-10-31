package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BLue2", group="Auto")
public class Blue2 extends LinearOpMode {
    public AutoMethods bot = new AutoMethods();





    //ОСНОВНАЯ ПРОГРАММА
    @Override
    public void runOpMode() throws InterruptedException {


        bot.initC(this);      //Инициализация
        bot.camStart(this);   //Запуск камеры
        bot.initIMU(this);//Запуск гироскопа
        bot.close();
        bot.sleep(350);
        waitForStart();
        bot.start();
        bot.getPos();

        bot.camStop();

        bot.drive(0, -4250, this, 3);

        if (bot.baza == 1) {
            bot.Telescope(400);
            bot.drive(-4000, 0, this, 3);
        }
        if (bot.baza == 2) {

            bot.Telescope(400);

        }
        if (bot.baza == 3) {

            bot.Telescope(400);
            bot.drive(4000, 0, this, 3);
        }
    }
}
