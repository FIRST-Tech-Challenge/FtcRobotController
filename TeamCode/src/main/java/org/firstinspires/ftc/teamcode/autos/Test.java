package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.chassis.MecanumChassis;

@Autonomous(name = "Test", group = "Sensor")
public class Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel();

    public void runOpMode() {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            // Start button is pressed

            moveForward(0.5,1000);
            moveBackward(0.5,1000);
            strafeLeft(0.5,1000);
            strafeRight(0.5,1000);
            turnLeft(0.5,1000);
            turnRight(0.5,1000);

            // End of auto
            break;
        }
    }

    // Basic functions

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }

    public void moveForward(double power, int time) {
        chassis.frontLeft.setPower(power);
        chassis.frontRight.setPower(power);
        chassis.backLeft.setPower(power);
        chassis.backRight.setPower(power);
        delay(time);
        chassis.frontLeft.setPower(0);
        chassis.frontRight.setPower(0);
        chassis.backLeft.setPower(0);
        chassis.backRight.setPower(0);
    }

    public void moveBackward(double power, int time) {
        chassis.frontLeft.setPower(-power);
        chassis.frontRight.setPower(-power);
        chassis.backLeft.setPower(-power);
        chassis.backRight.setPower(-power);
        delay(time);
        chassis.frontLeft.setPower(0);
        chassis.frontRight.setPower(0);
        chassis.backLeft.setPower(0);
        chassis.backRight.setPower(0);
    }

    public void strafeRight(double power, int time){
        chassis.frontLeft.setPower(power);
        chassis.frontRight.setPower(-power);
        chassis.backLeft.setPower(-power);
        chassis.backRight.setPower(power);
        delay(time);
        chassis.frontLeft.setPower(0);
        chassis.frontRight.setPower(0);
        chassis.backLeft.setPower(0);
        chassis.backRight.setPower(0);
    }

    public void strafeLeft(double power, int time){
        chassis.frontLeft.setPower(-power);
        chassis.frontRight.setPower(power);
        chassis.backLeft.setPower(power);
        chassis.backRight.setPower(-power);
        delay(time);
        chassis.frontLeft.setPower(0);
        chassis.frontRight.setPower(0);
        chassis.backLeft.setPower(0);
        chassis.backRight.setPower(0);
    }

    public void turnRight(double power, int time){
        chassis.frontLeft.setPower(power);
        chassis.frontRight.setPower(-power);
        chassis.backLeft.setPower(power);
        chassis.backRight.setPower(-power);
        delay(time);
        chassis.frontLeft.setPower(0);
        chassis.frontRight.setPower(0);
        chassis.backLeft.setPower(0);
        chassis.backRight.setPower(0);
    }

    public void turnLeft(double power, int time){
        chassis.frontLeft.setPower(-power);
        chassis.frontRight.setPower(power);
        chassis.backLeft.setPower(-power);
        chassis.backRight.setPower(power);
        delay(time);
        chassis.frontLeft.setPower(0);
        chassis.frontRight.setPower(0);
        chassis.backLeft.setPower(0);
        chassis.backRight.setPower(0);
    }
}
