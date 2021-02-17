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

@Autonomous(name="Autonomous_Proto", group="Autonomous")
@Disabled
public class Autonomous_Proto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private CRServo SoN;
    private Servo pushover;         //empties hopper
    private DcMotor spindoctor;     //spins wheels

    private ColorSensor CS;         //lower sensor
    private DistanceSensor DS;      //upper sensor
    double distance;
    double color;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        SoN = hardwareMap.get(CRServo.class, "SoN");
        spindoctor = hardwareMap.get(DcMotorEx.class, "spindoctor");
        pushover = hardwareMap.get(Servo.class, "pushover");
        CS = hardwareMap.get(ColorSensor.class, "cs");
        DS = hardwareMap.get(DistanceSensor.class, "ds");

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        SoN.setDirection(CRServo.Direction.FORWARD);
        spindoctor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 30.0)) {

            spindoctor.setPower(0);
            SoN.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
            pushover.setDirection(Servo.Direction.FORWARD);
            sleep(250);

            spindoctor.setPower(0);
            SoN.setPower(0);
            lf.setPower(1.0);
            rf.setPower(1.0);
            lb.setPower(1.0);
            rb.setPower(1.0);
            pushover.setDirection(Servo.Direction.FORWARD);
            sleep(500);

            if (color >= 0 && distance <= 0){


                // Autonomous A: four rings
                spindoctor.setPower(0);
                SoN.setPower(0);
                lf.setPower(0);
                rf.setPower(0);
                lb.setPower(0);
                rb.setPower(0);
                pushover.setDirection(Servo.Direction.FORWARD);
                sleep(1000);

            }else {

                // check for Autonomous B
                spindoctor.setPower(0);
                SoN.setPower(0);
                lf.setPower(0);
                rf.setPower(0);
                lb.setPower(0);
                rb.setPower(0);
                pushover.setDirection(Servo.Direction.FORWARD);
                sleep(1000);

                if (color >= 0 && distance > 0){

                    // Autonomous B: one ring
                    spindoctor.setPower(0);
                    SoN.setPower(0);
                    lf.setPower(0);
                    rf.setPower(0);
                    lb.setPower(0);
                    rb.setPower(0);
                    pushover.setDirection(Servo.Direction.FORWARD);
                    sleep(1000);

                }else {

                    //Autonomous C: no rings
                    spindoctor.setPower(0);
                    SoN.setPower(0);
                    lf.setPower(0);
                    rf.setPower(0);
                    lb.setPower(0);
                    rb.setPower(0);
                    pushover.setDirection(Servo.Direction.FORWARD);
                    sleep(1000);

                }
            }
            //function  VVV
            /*
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Distance (cm)",
            String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
             */

        }
    }
}