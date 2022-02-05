package org.firstinspires.ftc.teamcode.OnBotJava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Carousel.Carousel;
import org.firstinspires.ftc.teamcode.Chassis.Drivechain;

@Autonomous
public class PosFourSPECIAL extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivechain AutonomousDC = new Drivechain(hardwareMap);

        waitForStart();


        AutonomousDC.resetTicks();
        //Forward
        AutonomousDC.moveRobot(-925, 925, -925, 925, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Counterclockwise 30 degrees
        AutonomousDC.moveRobot( 300, 300, 300, 300, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Clockwise 30 degrees
        AutonomousDC.moveRobot(-300, -300, -300, -300, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Forward
        AutonomousDC.moveRobot(300, 300, 300, 300, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);



    }
}

