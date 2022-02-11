package org.firstinspires.ftc.teamcode.OnBotJava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Carousel.Carousel;
import org.firstinspires.ftc.teamcode.Chassis.Drivechain;

@Autonomous
public class PosThreeSPECIAL extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivechain AutonomousDC = new Drivechain(hardwareMap);
        Carousel AutonomousCS = new Carousel(hardwareMap);
        AutonomousDC.calibrateGyro();
        AutonomousDC.resetGyro();
        AutonomousDC.resetTicks();
        waitForStart();

        // TODO: Scan where the duck is

        //Move forward 1 tile
        AutonomousDC.moveRobot(-925, 925, -925, 925, .5, .5, -.5, -.5);
        while (AutonomousDC.isbusy()) {
            sleep(1000);
        }

        //TODO: Intake the duck

        //Clockwise 30 degrees
        AutonomousDC.resetTicks();
        AutonomousDC.turnDeg(30.0f, telemetry);
        AutonomousDC.resetTicks();

        //TODO: Outtake the duck on the hub

        //Counter clockwise 30 degrees
        AutonomousDC.resetTicks();
        AutonomousDC.turnDeg(-30.0f, telemetry);
        AutonomousDC.resetTicks();

        //Move forward 1 tile (to get out of the way)
        AutonomousDC.moveRobot(-925, 925, -925, 925, .5, .5, -.5, -.5);

        //End of program

    }
}