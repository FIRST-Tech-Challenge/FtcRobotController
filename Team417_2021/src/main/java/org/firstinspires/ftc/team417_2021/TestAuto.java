package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Duck Parking")
public class TestAuto extends MasterAutonomous {

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
        /*pivot(10 * allianceSide, 0.5);
        move3(6, 0.7);
        pivot(20 * allianceSide)
        pivot(-60 * allianceSide, 0.7);
        move3(10, 0.7);*/

        move3(-6, 0.7);
        pivot(90 * allianceSide, 0.7);
        move3(22, 0.7);
        pivot(0, 0.7);
        move3(6, 0.7);
        carouselMotor.setPower(0.8 * allianceSide);
        sleep(10000);
        carouselMotor.setPower(0.0);
        move3(-5, 0.7);
        pivot(-90 * allianceSide, 0.7);
        move3(130, 1.0);



    }
}
