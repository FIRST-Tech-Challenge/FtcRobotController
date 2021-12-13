package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Autonomous(group = "Test")
public class Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel(Color.BLUE);

    public void runOpMode() throws InterruptedException {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);

        waitForStart();
        // Start button is pressed

        chassis.moveForwardByTime(0.5,1000);
        chassis.moveBackwardByTime(0.5,1000);
        chassis.strafeLeftByTime(0.5,1000);
        chassis.strafeRightByTime(0.5,1000);
        chassis.rotateToGlobalAngle(90,0.5);
        chassis.rotateToGlobalAngle(0,0.5);
    }

}
