package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testServos extends LinearOpMode {

    Servo FLServo, FRServo, BRServo, BLServo;

    @Override
    public void runOpMode() {

        initRobot();
        waitForStart();

        while (opModeIsActive()) {

            //inits pow
            double flPow;
            double frPow;
            double blPow;
            double brPow;

            //sets booleans
            int set = 0;
            if (gamepad1.a)
                set = 0;
            if(gamepad1.b)
                set = 0;
            if(gamepad1.x)
                set = 0;
            if(gamepad1.y)
                set = 0;

            //sets pow
            flPow = set;
            frPow = set;
            blPow = set;
            brPow = set;


            //sets servos
            FLServo.setPosition(flPow);
            FRServo.setPosition(frPow);
            BLServo.setPosition(blPow);
            BRServo.setPosition(brPow);

            //sets telemetry
            addTelemetry(flPow, frPow, blPow, brPow);
        }
    }

    public void addTelemetry(double flPow, double frPow, double blPow, double brPow) {
        telemetry.addData("FL: ", flPow);
        telemetry.addData("FR: ", frPow);
        telemetry.addData("BL: ", blPow);
        telemetry.addData("BR: ", brPow);
        telemetry.update();
    }

    public void initRobot() {

        FLServo = hardwareMap.get(Servo.class, "FLServo");
        FRServo = hardwareMap.get(Servo.class, "FRServo");
        BLServo = hardwareMap.get(Servo.class, "BLServo");
        BRServo = hardwareMap.get(Servo.class, "BRServo");

        FLServo.setDirection(Servo.Direction.FORWARD);
        FRServo.setDirection(Servo.Direction.FORWARD);
        BLServo.setDirection(Servo.Direction.FORWARD);
        BRServo.setDirection(Servo.Direction.FORWARD);

        FLServo.setPosition(0);
        FRServo.setPosition(0);
        BRServo.setPosition(0);
        BLServo.setPosition(0);

        FLServo.scaleRange(0, 1.0);
        FRServo.scaleRange(0, 1.0);
        BLServo.scaleRange(0, 1.0);
        BRServo.scaleRange(0, 1.0);
    }
}
