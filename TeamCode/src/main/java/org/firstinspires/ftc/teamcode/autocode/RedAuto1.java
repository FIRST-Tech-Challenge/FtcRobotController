package org.firstinspires.ftc.teamcode.autocode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.PlaceLinePixel;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "RedAuto1")

public class RedAuto1 extends PlaceLinePixel {

    @Override

    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        try {

            initTfod();

            waitForStart();

            if (opModeIsActive()) {
                Grab();
                TimeUnit.MILLISECONDS.sleep(250);

                RobotMoveFarwardHalf();

                RobotStop();

                armUp();

                TimeUnit.SECONDS.sleep(2);
                telemetryTfod();
                telemetry.update();

                if (Location1 == true) {
                    PixelLocation1();
                } else if (Location2 == true) {
                    PixelLocation2();
                    frontLeftMotor.setPower(.2);
                    frontRightMotor.setPower(.2);
                    backLeftMotor.setPower(.2);
                    backRightMotor.setPower(.2);
                    TimeUnit.MILLISECONDS.sleep(200);
                    RobotStop();
                } else if (Location3 == true) {
                    PixelLocation3();
                } else {
                    TimeUnit.SECONDS.sleep(2);
                    telemetryTfod();
                    telemetry.update();
                    if (Location1 == true) {
                        PixelLocation1();
                    } else if (Location2 == true) {
                        PixelLocation2();
                        frontLeftMotor.setPower(.2);
                        frontRightMotor.setPower(.2);
                        backLeftMotor.setPower(.2);
                        backRightMotor.setPower(.2);
                        TimeUnit.MILLISECONDS.sleep(200);
                        RobotStop();
                    } else if (Location3 == true) {
                        PixelLocation3();
                    } else {
                        Location3 = true;
                        PixelLocation3();
                    }
                }

                TimeUnit.MILLISECONDS.sleep(500);

                RedLocation1();

                if (Location1 == true) {
                    BoardPixel1();
                    RobotMoveBackwardHalf();
                    RobotStrafeRightHalf();
                    TimeUnit.MILLISECONDS.sleep(1000);
                    RobotMoveFarwardHalf();
                } else if (Location2 == true) {
                    BoardPixel2();
                    RobotMoveBackwardHalf();
                    RobotStrafeRightHalf();
                    TimeUnit.MILLISECONDS.sleep(900);
                    RobotMoveFarwardHalf();
                } else if (Location3 == true) {
                    BoardPixel3();
                    RobotMoveBackwardHalf();
                    RobotStrafeRightHalf();
                    TimeUnit.MILLISECONDS.sleep(700);
                    RobotMoveFarwardHalf();
                }

                TimeUnit.MILLISECONDS.sleep(100);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }
}
