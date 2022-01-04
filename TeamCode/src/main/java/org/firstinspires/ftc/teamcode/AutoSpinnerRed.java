package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name="AutoTestLeftRed", group="Autonomous")
//@Disabled
public class AutoSpinnerRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;  //left front wheel
    private DcMotor rf = null;  //right front wheel
    private DcMotor lb = null;  //left back wheel
    private DcMotor rb = null;  //right back wheel
    private DcMotor tower1 = null; //arm motor 1
    private DcMotor tower2 = null; //arm motor 2
    private Servo clawservo = null; //clawservo
    private CRServo duckspinner = null; // the duck spinny thingy




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
        duckspinner = hardwareMap.get(CRServo.class, "duckspinner");

        double sidemult = 1.0; //Red side = 1.0 Blue = -1.0


        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);
        tower1.setDirection(DcMotor.Direction.FORWARD);
        tower2.setDirection(DcMotor.Direction.FORWARD);
        duckspinner.setDirection(CRServo.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 30.0)) {


        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(0.5);
        rf.setPower(0.5);
        lb.setPower(0.5);
        rb.setPower(0.5);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(500); // Go forward towards goal


        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(0.5 * sidemult);
        rf.setPower(-0.5 * sidemult);
        lb.setPower(0.5 * sidemult);
        rb.setPower(-0.5 * sidemult);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(250); // Turn right 45 towards goal, and away from spinner

        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(0.5);
        rf.setPower(0.5);
        lb.setPower(0.5);
        rb.setPower(0.5);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(250); // Forward towards goal

        tower1.setPower(0.5);
        tower2.setPower(0.5);
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(250); // Lower arm onto goal

        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        clawservo.setPosition(0);
        duckspinner.setPower(0);
        sleep(250); // Open claw and drop block onto goal

        tower1.setPower(-0.5);
        tower2.setPower(-0.5);
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(250); // Raise arm and close claw

        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(-0.5);
        rf.setPower(-0.5);
        lb.setPower(-0.5);
        rb.setPower(-0.5);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(750); // Go backwards towards Carousel

        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        clawservo.setPosition(0.75);
        duckspinner.setPower(1);
        sleep(7000); // Spin duck spinner

        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(-0.5 * sidemult);
        rf.setPower(0.5 * sidemult);
        lb.setPower(-0.5 * sidemult);
        rb.setPower(0.5 * sidemult);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(250); // Turn left 90

        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(0.5);
        rf.setPower(0.5);
        lb.setPower(0.5);
        rb.setPower(0.5);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(250); // Go forward

        tower1.setPower(0);
        tower2.setPower(0);
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        clawservo.setPosition(0.75);
        duckspinner.setPower(0);
        sleep(25000); // Stop
        }
    }
}