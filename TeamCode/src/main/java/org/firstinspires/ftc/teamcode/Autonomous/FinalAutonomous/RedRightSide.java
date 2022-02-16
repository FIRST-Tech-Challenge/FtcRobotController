package org.firstinspires.ftc.teamcode.Autonomous.FinalAutonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Carousel.Carousel;
import org.firstinspires.ftc.teamcode.Chassis.Drivechain;

@Autonomous
public class RedLeftSide extends LinearOpMode {

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
        AutonomousDC.turnDeg(-40.0f, telemetry);
        sleep(2000);
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(400, -400, 400, -400, 1, 1, -1, -1);
        sleep(2000);
        AutonomousDC.resetTicks();
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        AutonomousDC.resetTicks();

        //Outtake stuff

        AutonomousDC.turnDeg(220.0f, telemetry);
        sleep(2000);
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(200, -200, 200, -200, 1, 1, -1, -1);
        sleep(2000);
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        AutonomousDC.resetTicks();
        AutonomousDC.turnDeg(-90.0f, telemetry);
        sleep(2000);
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(700, -700, 700, -700, 1, 1, -1, -1);
        sleep(2000);
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
    }
}

