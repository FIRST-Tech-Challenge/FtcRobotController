package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.chassis.MecanumChassis;

@Autonomous(name = "Spin Carousel", group = "Sensor")
public class SpinCarousel extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel();

    public void runOpMode() {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            // Start button is pressed

            carousel.turnCarousel();
            delay(2000);
            carousel.carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            carousel.carousel.setPower(0.55);
            delay(200);
            carousel.carousel.setPower(-0.55);
            delay(300);
            carousel.carousel.setPower(0);


            // End of auto
            break;
        }
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
