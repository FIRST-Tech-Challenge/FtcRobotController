package org.firstinspires.ftc.teamcode.Autonomous.CodeBlue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Chassis.Drivechain;
import org.firstinspires.ftc.teamcode.Carousel.Carousel;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Outtake.Outtake;

public class AutonomousCodeBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Intake AutonomousIntake = new Intake(hardwareMap);
        Outtake AutonomousOuttake = new Outtake(hardwareMap);
        Carousel AutonomousCarousel = new Carousel(hardwareMap);
        Drivechain AutonomousDC = new Drivechain(hardwareMap);

        waitForStart();

        // TODO, step 1

        //Step 2: DONE
//        AutonomousOuttake.FlapOpen();
        AutonomousCarousel.CarouselAutonomous();
        //TODO, step 3

        //Step 4: DONE
        //AutonomousCarousel.CarouselHandler();

        AutonomousDC.moveRobot(telemetry, 1000, -1000, 1000, -1000, 0.25, -0.25, 0.25, -0.25);

        //TODO, step 5/6


    }
}
