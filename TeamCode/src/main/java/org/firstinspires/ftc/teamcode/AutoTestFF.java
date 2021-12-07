package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

    @Autonomous(name="AutoTestFF", group="Autonomous")
//@Disabled
    public class AutoTestFF extends LinearOpMode {
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor lf = null;  //left front wheel
        private DcMotor rf = null;  //right front wheel
        private DcMotor lb = null;  //left back wheel
        private DcMotor rb = null;  //right back wheel
        private DcMotor tower1 = null; //arm motor 1
        private DcMotor tower2 = null; //arm motor 2
        private Servo clawservo = null; //clawservo

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Ready to run");
            telemetry.update();

            lf = hardwareMap.get(DcMotor.class, "lf");
            rf = hardwareMap.get(DcMotor.class, "rf");
            lb = hardwareMap.get(DcMotor.class, "lb");
            rb = hardwareMap.get(DcMotor.class, "rb");
            tower1 = hardwareMap.get(DcMotor.class, "tower1");
            tower2 = hardwareMap.get(DcMotor.class, "tower2");
            clawservo = hardwareMap.get(Servo.class,"clawservo");

            lf.setDirection(DcMotor.Direction.REVERSE);
            rf.setDirection(DcMotor.Direction.FORWARD);
            lb.setDirection(DcMotor.Direction.REVERSE);
            rb.setDirection(DcMotor.Direction.FORWARD);
            tower1.setDirection(DcMotor.Direction.FORWARD);
            tower2.setDirection(DcMotor.Direction.REVERSE);

            waitForStart();
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 30.0)) {

                tower1.setPower(0);
                tower2.setPower(0);
                lf.setPower(0);
                rf.setPower(0);
                lb.setPower(0);
                rb.setPower(0);
                clawservo.setPosition(0);
                sleep(250);


            }
        }
    }
