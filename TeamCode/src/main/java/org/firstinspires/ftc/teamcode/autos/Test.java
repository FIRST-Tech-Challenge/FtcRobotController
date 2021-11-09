package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Autonomous(name = "Test", group = "Sensor")
public class Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel();

    public void runOpMode() {
        chassis.init(hardwareMap, true);
        carousel.init(hardwareMap, true);

        waitForStart();
        while (opModeIsActive()) {
            // Start button is pressed

            chassis.moveForwardByTime(0.5,1000);
            chassis.moveBackwardByTime(0.5,1000);
            chassis.strafeLeftByTime(0.5,1000);
            chassis.strafeRightByTime(0.5,1000);
            chassis.turnLeftByTime(0.5,1000);
            chassis.turnRightByTime(0.5,1000);

            // End of auto
            break;
        }
    }

}
