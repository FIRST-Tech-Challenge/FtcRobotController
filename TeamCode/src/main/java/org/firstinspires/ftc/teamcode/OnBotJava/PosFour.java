package org.firstinspires.ftc.teamcode.OnBotJava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Carousel.Carousel;
import org.firstinspires.ftc.teamcode.Chassis.Drivechain;

@Autonomous
public class PosFour extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivechain AutonomousDC = new Drivechain(hardwareMap);
        Carousel AutonomousCS = new Carousel(hardwareMap);
        AutonomousDC.calibrateGyro();
        AutonomousDC.resetGyro();
        AutonomousDC.resetTicks();
        waitForStart();


//        //Forward
//        AutonomousDC.moveRobot(-925, 925, -925, 925, 0.25, 0.25, 0.25, 0.25);
//        sleep(3000);
//
//        //Counterclockwise 30 degrees
//        AutonomousDC.moveRobot( 300, 300, 300, 300, 0.25, 0.25, 0.25, 0.25);
//        sleep(3000);
//
//        //Clockwise 30 degrees
//        AutonomousDC.moveRobot(-300, -300, -300, -300, 0.25, 0.25, 0.25, 0.25);
//        sleep(3000);
//
//        //Forward
//        AutonomousDC.moveRobot(-925, 925, -925, 925, 0.25, 0.25, 0.25, 0.25);
//        sleep(3000);

        sleep(2500);
        AutonomousDC.moveRobot(800, -800, 800, -800, .5, .5, -.5, -.5);
        sleep(2000);
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        AutonomousDC.resetTicks();
        AutonomousDC.turnDeg(-45.0f, telemetry);
        AutonomousDC.resetTicks();
        sleep(1000);
        AutonomousDC.moveRobot(200, -200, 200, -200, .5, .5, -.5, -.5);
        sleep(2000);

        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        AutonomousDC.resetTicks();
        AutonomousDC.turnDeg(135.0f, telemetry);
        AutonomousDC.resetTicks();
        sleep(1000);
        AutonomousDC.moveRobot(1000, -1000, 1000, -1000, .5, .5, -.5, -.5);
        sleep(2000);

        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        AutonomousDC.resetTicks();
        AutonomousDC.turnDeg(-180.0f, telemetry);
        AutonomousDC.resetTicks();
        sleep(1000);
        AutonomousDC.moveRobot(5000, -5000, 5000, -5000, .5, .5, -.5, -.5);
        sleep(2000);
        AutonomousCS.CarouselAutonomous(1000,0.3);
        sleep(2000);
        AutonomousDC.resetTicks();
        AutonomousDC.turnDeg(135.0f, telemetry);
        AutonomousDC.resetTicks();
        sleep(1000);
        AutonomousDC.moveRobot(400, -400, 400, -400, .5, .5, -.5, -.5);
        sleep(2000);
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }


    }
}

