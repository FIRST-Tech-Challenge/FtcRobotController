package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueRight", group="Auto")
//@Disabled
public class BlueRight extends LinearOpMode {

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

        bot.Telescope(1600);
        bot.drive(-3500, 0, this, 3);// дИАГональ
        bot.drive(0, -6000, this, 3);//едем вперёд
        bot.drive(-1550, -1550, this, 3);//едем вперёд
        sleep(500);
        bot.open();
        sleep(500);
        bot.drive(0, 500, this, 1.5);//едем назад


        if (bot.baza == 1) {
            bot.turn(90,this);// поворачиваемся направо
            bot.drive(0, 1500,this,3);

        }
        if (bot.baza == 2) {

            bot.drive(2000, 0, this,3);
        }
        if (bot.baza == 3) {

            bot.drive(4600, 0, this,3);
        }
    }
}