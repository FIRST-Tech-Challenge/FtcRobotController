package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Duck Parking")
public class DuckParkingAuto extends MasterAutonomous {

    int allianceSide = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addLine("ready");
        telemetry.update();
        while (!opModeIsActive()) {
            if (gamepad1.a) {
                allianceSide *= -1;
            }
            telemetry.addLine("1 for blue, -1 for red");
            telemetry.addData("Alliance", allianceSide);
            telemetry.update();
            idle();
        }

        waitForStart();

        if (allianceSide == 1) {
            moveInches(-6, 0.7);
            pivot(90, 0.7);
            moveInches(22, 0.7);
            pivot(0, 0.7);
            moveInches(8, 0.7);
            carouselMotor.setPower(0.8);
            sleep(10000);
            carouselMotor.setPower(0.0);
            moveInches(-5, 0.7);
            pivot(-90, 0.7);
            moveInches(100, 1.0);
        } else {
            moveInches(-3, 0.7);
            pivot(-90, 0.7);
            moveInches(24, 0.7);
            carouselMotor.setPower(0.8);
            sleep(10000);
            carouselMotor.setPower(0.0);
            moveInches(-4, 0.7);
            pivot(90, 0.7);
            moveInches(100, 1.0);
        }
    }
}
