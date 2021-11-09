package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Autonomous(name = "Park", group = "Sensor")
public class Park extends LinearOpMode {
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel();

    public void runOpMode() {
        chassis.init(hardwareMap, true);
        carousel.init(hardwareMap, true);

        waitForStart();
        while (opModeIsActive()) {
            // Start button is pressed

            // Drive into the warehouse
            chassis.moveForwardWithEncoders(0.6,1000);


            // End of auto
            break;
        }
    }
}
