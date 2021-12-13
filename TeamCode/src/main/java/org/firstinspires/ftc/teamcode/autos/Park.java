package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Autonomous(name = "Park", group = "Autonomous")
public class Park extends LinearOpMode {
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel(Color.BLUE);

    public void runOpMode() {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);

        waitForStart();
        // Start button is pressed

        // Drive into the warehouse
        chassis.moveForwardWithEncoders(0.6,1000);
    }
}
