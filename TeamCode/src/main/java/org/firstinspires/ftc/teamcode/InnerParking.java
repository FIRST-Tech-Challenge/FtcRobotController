package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

@Autonomous(name="InnerParking", group="Autonomous")
//@Disabled
public class InnerParking extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private CRServo SoN;
    private Servo pushover = null;  //empties hopper
    private Servo wobble = null;    //wobble goal arm
    private DcMotor spindoctorL;    //spins wheels
    private DcMotor spindoctorR;    //spins wheels
    private DcMotor factory;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        SoN = hardwareMap.get(CRServo.class, "SoN");
        pushover = hardwareMap.get(Servo.class, "pushover");
        wobble = hardwareMap.get(Servo.class, "wobble");
        spindoctorL = hardwareMap.get(DcMotor.class, "spinL");
        spindoctorR = hardwareMap.get(DcMotor.class, "spinR");
        factory = hardwareMap.get(DcMotor.class,"factory");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        SoN.setDirection(CRServo.Direction.FORWARD);
        spindoctorL.setDirection(DcMotorSimple.Direction.FORWARD);
        spindoctorR.setDirection(DcMotorSimple.Direction.REVERSE);
        factory.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 30.0)) {

            spindoctorL.setPower(0);
            spindoctorR.setPower(0);
            SoN.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
            factory.setPower(0);
            sleep(250);

            spindoctorL.setPower(0);
            spindoctorR.setPower(0);
            lf.setPower(0.4);
            rf.setPower(0.4);
            lb.setPower(0.4);
            rb.setPower(0.4);
            sleep(1400);

            spindoctorL.setPower(0);
            spindoctorR.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
            sleep(2500000);

        }
    }
}
