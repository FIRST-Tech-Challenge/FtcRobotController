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

@Autonomous(name="Auto_Red_Right", group="Autonomous")
//@Disabled
public class Auto_Red_Right extends LinearOpMode {
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

    private ColorSensor CS;
    private DistanceSensor DS;
    double distance;
    double color;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    int colorDistance;

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
        CS = hardwareMap.get(ColorSensor.class, "cs");
        DS = hardwareMap.get(DistanceSensor.class, "ds");

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

            if(CS.red() >= 240 && CS.green() >= 60 && CS.green() <= 170){
                color = 1;
            }else {
                color = 0;
            }

            if (DS.getDistance(DistanceUnit.CM) <= 5){
                distance = 1;
            }else {
                distance = 0;
            }

            spindoctorL.setPower(0);
            spindoctorR.setPower(0);
            SoN.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
            factory.setPower(0);
            colorDistance = 0;
            sleep(250);

            spindoctorL.setPower(0);
            spindoctorR.setPower(0);
            lf.setPower(-1.0);
            rf.setPower(1.0);
            lb.setPower(-1.0);
            rb.setPower(1.0);
            sleep(250);

            spindoctorL.setPower(0);
            spindoctorR.setPower(0);
            lf.setPower(-1.0);
            rf.setPower(-1.0);
            lb.setPower(-1.0);
            rb.setPower(-1.0);
            sleep(2000);

            spindoctorL.setPower(0);
            spindoctorR.setPower(0);
            lf.setPower(-1.0);
            rf.setPower(1.0);
            lb.setPower(-1.0);
            rb.setPower(1.0);
            sleep(250);

            spindoctorL.setPower(0);
            spindoctorR.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
            colorDistance = 1;
            sleep(250);

            if (color == 1 && distance == 1){


                // Autonomous A: four rings; farthest box
                spindoctorL.setPower(0);
                spindoctorR.setPower(0);
                lf.setPower(0);
                rf.setPower(0);
                lb.setPower(0);
                rb.setPower(0);
                sleep(1000);

            }else {

                if (color == 1 && distance == 0){

                    // Autonomous B: one ring; middle box
                    spindoctorL.setPower(0);
                    spindoctorR.setPower(0);
                    lf.setPower(0);
                    rf.setPower(0);
                    lb.setPower(0);
                    rb.setPower(0);
                    sleep(1000);

                }else {

                    //Autonomous C: no rings; nearest box
                    spindoctorL.setPower(0);
                    spindoctorR.setPower(0);
                    lf.setPower(0);
                    rf.setPower(0);
                    lb.setPower(0);
                    rb.setPower(0);
                    sleep(1000);

                }
            }
            //function  VVV

            if (colorDistance == 1){
                Color.RGBToHSV((int) (CS.red() * SCALE_FACTOR),
                        (int) (CS.green() * SCALE_FACTOR),
                        (int) (CS.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", DS.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", CS.alpha());
                telemetry.addData("Red  ", CS.red());
                telemetry.addData("Green", CS.green());
                telemetry.addData("Blue ", CS.blue());
                telemetry.addData("Hue", hsvValues[0]);
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });
                telemetry.update();}


        }
    }
}