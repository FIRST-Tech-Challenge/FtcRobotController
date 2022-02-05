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

        Drivechain AutonomousDC = new Drivechain(hardwareMap);

        waitForStart();

        AutonomousDC.resetTicks();

        //Clockwise 90 Degrees
        AutonomousDC.moveRobot(-925, -925, -925, -925, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Forward Three Tiles
        AutonomousDC.moveRobot( -3000, 3000, -3000, 3000, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //180 Degrees Rotate
        AutonomousDC.moveRobot(-1800, -1800, -1800, -1800, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Forward Three Tiles
        AutonomousDC.moveRobot( -3000, 3000, -3000, 3000, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Clockwise 90 Degrees
        AutonomousDC.moveRobot(-925, -925, -925, -925, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Forward One Tile
        AutonomousDC.moveRobot(-925, 925, -925, 925, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Counterclockwise 30 degrees
        AutonomousDC.moveRobot( 925, 925, 925, 925, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Forward One Tile
        AutonomousDC.moveRobot(-925, 925, -925, 925, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

    }
}

