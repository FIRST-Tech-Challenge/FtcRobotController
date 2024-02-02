package org.firstinspires.ftc.teamcode.autocode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PlaceLinePixel;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "test")

public class test extends PlaceLinePixel {

    @Override

    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        linearGripper = hardwareMap.get(Servo.class, "linearGripper");

        try {

            initTfod();

            waitForStart();
            double index = 0;

            if (opModeIsActive()) {
                test();
                RobotMoveFarward();
                TimeUnit.MILLISECONDS.sleep(1000);
                RobotStop();
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }
}