package org.firstinspires.ftc.teamcode.OnBotJava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Carousel.Carousel;
import org.firstinspires.ftc.teamcode.Chassis.Drivechain;

@Autonomous
public class PosTwoVAR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Carousel AutonomousCarousel = new Carousel(hardwareMap);
        Drivechain AutonomousDC = new Drivechain(hardwareMap);

        waitForStart();



        /*
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(telemetry, -925, -925, -925, -925, 0.25, 0.25, 0.25, 0.25);

        sleep(3000);

        AutonomousCarousel.reset();
        AutonomousCarousel.CarouselAutonomous(1600, 0.3);

        sleep(3000);

        //AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(telemetry, 0, 0, 0, 0, 0.25, 0.25, 0.25, 0.25);
        */

        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(-700, 700, -700, 700, 0.5, 0.5, -0.5, -0.5); //forward
        while(AutonomousDC.isbusy()){
            sleep(100);
        }
        sleep(1000);

        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(1000, 1000, 1000, 1000, 0.8, 0.5, 0.5, 0.5); //rotate c
        while(AutonomousDC.isbusy()){
            sleep(100);
        }
        sleep(2000);
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(1200, 1200, -1200, 1200, 0.5, 0.5, -0.5, -0.5); //forward
        while(AutonomousDC.isbusy()){
            sleep(100);
        }
        sleep(1000);
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(1000, 1000, 1000, 1000, 0.8, 0.5, 0.5, 0.5);  //forward
        while(AutonomousDC.isbusy()){
            sleep(100);
        }
        sleep(1000);
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(1200, 1200, -1200, 1200, 0.5, 0.5, -0.5, -0.5); //forward
        while(AutonomousDC.isbusy()){
            sleep(100);
        }
        // AutonomousDC.resetTicks();
        // AutonomousDC.moveRobot(1000, 1000, 1000, 1000, 0.5, 0.5, 0.5, 0.5); //forward
        // while(AutonomousDC.isbusy()){
        //     sleep(100);
        // }
        // sleep(1000);
        //  AutonomousDC.resetTicks();
        // AutonomousDC.moveRobot(-800, 800, -800, 800, 0.5, 0.5, -0.5, -0.5); //forward
        // while(AutonomousDC.isbusy()){
        //     sleep(100);
        // }
        // sleep(1000);
        // sleep(30);

        // AutonomousDC.resetTicks();
        // AutonomousDC.moveRobot(0, 3900, 0, 3900, 0.6, 0.6, 0.6, 0.6);
        //  sleep(4500);
        // AutonomousDC.showTicks(telemetry);


        // AutonomousDC.resetTicks();
        // AutonomousDC.moveRobot(-60, 60, -60, 60, 0.07, 0.07, 0.07, 0.07);

        // AutonomousCarousel.CarouselAutonomous(-2000, 0.3);

        // sleep(3000);

        // //AutonomousDC.resetTicks();
        // //AutonomousDC.moveRobot(telemetry, 0, 0, 0, 0, 0.25, 0.25, 0.25, 0.25);


        // sleep(5000);

        // AutonomousCarousel.reset();



    }
}