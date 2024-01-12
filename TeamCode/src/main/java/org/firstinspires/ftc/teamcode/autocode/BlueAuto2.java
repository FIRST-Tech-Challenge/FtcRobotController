package org.firstinspires.ftc.teamcode.autocode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.utils.PlaceLinePixel;

@Autonomous(name = "BlueAuto2")

public class BlueAuto2 extends PlaceLinePixel{

    @Override

    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        try {

            initTfod();

            waitForStart();

            if (opModeIsActive()) {
                TimeUnit.MILLISECONDS.sleep(250);

                RobotMoveFarward();
                TimeUnit.MILLISECONDS.sleep(250);

                RobotStop();
                TimeUnit.MILLISECONDS.sleep(250);

                TimeUnit.SECONDS.sleep(5);
                telemetryTfod();
                telemetry.update();
                TimeUnit.SECONDS.sleep(1);

                if (Location1 == true) {
                    PixelLocation1();
                } else if (Location2 == true) {
                    PixelLocation2();
                } else if (Location3 == true) {
                    PixelLocation3();
                }
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }
}
