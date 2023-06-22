package org.firstinspires.ftc.teamcode.Mechanism;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
public class ConeTransporter extends HWMap {
    /*
    This is the general explanation for this class:
    liftPos0 = position to pick up the cone
    liftPos1 = lowest junction
    liftPos2 = medium junction
    liftPos3 = high junction

    gripperClose = gripper closed i.e. not grasping the cone - NOT ACTUALLY USED
    gripperOpen = gripper expanded i.e. grasping the cone
    */


    //    public Map<LinearSlidesLevels, Double> linearSlidesLevels;
    public double V15 = 0;

    // Tele-Op
    public static double LINEAR_SLIDES_LOW = 372.5;// 13.5 inches converted to mm(low junction)
    public static double LINEAR_SLIDES_MEDIUM = 632.5;// 23.5 inches converted to mm(medium junction)
    //Linear slide high used to be 895
    public static double LINEAR_SLIDES_HIGH = 901;// 33.5 inches converted to mm(high junction) 2349
    public static double LINEAR_SLIDES_NORM = 105;
    public static double LINEAR_SLIDES_INIT = 30;
    public static double LINEAR_SLIDES_IN_CONE = 0;
    public static double LINEAR_SLIDES_CURRENT = LINEAR_SLIDES_NORM;
    public static double ticks;
    //Autonomous
    private static final double inFactor_MM = 125;
    public  boolean automation = false;

    public static double AUTO_LINEAR_SLIDES_11 = 133.35;// 5.25 inches converted to mm(1 stack) + 10 mm to be above the cone
    public static double AUTO_LINEAR_SLIDES_12 = 165.1;// 6.5 inches converted to mm(2 stack) + 10 mm to be above the cone
    public static double AUTO_LINEAR_SLIDES_13 = 196.85;// 7.75 inches converted to mm(3 stack) + 10 mm to be above the cone
    public static double AUTO_LINEAR_SLIDES_14 = 228.6;// 9 inches converted to mm(4 stack) + 10 mm to be above the cone
    public static double AUTO_LINEAR_SLIDES_15 = 248;// 10.25 inches converted to mm(5 stack) + 10 mm to be above the cone
    public static double AUTO_LINEAR_SLIDES_11_IN_CONE = AUTO_LINEAR_SLIDES_11 - inFactor_MM;
    public static double AUTO_LINEAR_SLIDES_12_IN_CONE = AUTO_LINEAR_SLIDES_12 - inFactor_MM;
    public static double AUTO_LINEAR_SLIDES_13_IN_CONE = AUTO_LINEAR_SLIDES_13 - inFactor_MM;
    public static double AUTO_LINEAR_SLIDES_14_IN_CONE = AUTO_LINEAR_SLIDES_14 - inFactor_MM;
    public static double AUTO_LINEAR_SLIDES_15_IN_CONE = AUTO_LINEAR_SLIDES_15 - inFactor_MM;
    public static double ticksPerRotation = 384.5;// 435 RPM motor 5202 GoBilda TPR
    public static int ticksAsInt;

    //GRIPPER______________________________________________________________________________________
    public double gripperPosition;

    //LINEAR SLIDES________________________________________________________________________________

    public int riseLevel = 0;
    public int posLevel = 0;
    public static float diameterOfSpool = 34f;
    public float linearSlidesSpeed = 1f;
    public int level;
    public ArrayList<Double> stackLevel = new ArrayList<Double>();
    public ArrayList<String> telemetryLevel = new ArrayList<String>();
    public int arrayListIndex = -1;
    public String stackTelemetry;

    // Servos on encoder wheels for retracting and unretracting them
    public double LEFT_RETRACT_POS = 0.7;
    public double LEFT_UNRETRACT_POS = 0.25;
    public double RIGHT_RETRACT_POS = 0.0;
    public double RIGHT_UNRETRACT_POS = 0.6;
    public double FRONT_RETRACT_POS = 0.5;
    public double FRONT_UNRETRACT_POS = 1.0;

    public static double p = 0.008, i = 0, d = 0.00012 ;
    public double f;
    public static double h = 0.000025, b = 0.01;
    private PIDController controller = new PIDController(p, i, d);

    public static int target = 0;

    private final double ticks_in_degrees = 384.5/360;

    public static boolean zeroMode = false;

    private ElapsedTime timer = new ElapsedTime();

    //public RevBlinkinLedDriver blinkin;

    public boolean slideDisplay = false;

    public static ElapsedTime ledTimer = new ElapsedTime();


    public ConeTransporter(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
    }

    public void init() {
        //linearSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setGripperPosition(1.0);
        setHeight(0);
        retractOdometryServos();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        zeroMode = false;
    }

    public void setGripperPosition(double position) {
        gripper.setPosition(position);
    }

    public static int equate(double height) {
        ticks = ticksPerRotation * (height / (diameterOfSpool * Math.PI));
        ticksAsInt = (int) ticks;
        return ticksAsInt;
    }


    public static void setHeight(int level) {
        target = level;
    }


    public void zeroSlides(){
        if (!zeroMode){
            timer.reset();
        }
        if (zeroMode) {
            if (!(timer.time(TimeUnit.SECONDS) > 1.5)) {
                linearSlides.setPower(-.5);
            }
            if (linearSlides.getCurrent(CurrentUnit.MILLIAMPS) > 2000 & timer.time(TimeUnit.SECONDS) > 2){
                target = 0;
                linearSlides.setPower(0);

            }
            if (timer.time(TimeUnit.SECONDS) > 3) {
                linearSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                zeroMode = false;
            }
        }
    }


    public void coneSense() throws InterruptedException {

            if((gripperCS.red() >= 3000 || gripperCS.blue() >= 3000) && automation){
                linearSlides.setPower(0);
                setGripperPosition(0.75);
                sleep(75);
                target = equate(AUTO_LINEAR_SLIDES_15);
                automation = false;
            } else if (automation){
                target = equate(0);
            }
    }
    public void retractOdometryServos() {
        leftServo.setPosition(LEFT_RETRACT_POS);
        rightServo.setPosition(RIGHT_RETRACT_POS);
        //frontServo.setPosition(FRONT_RETRACT_POS);
    }

    public void unretractOdometryServos() {
        leftServo.setPosition(LEFT_UNRETRACT_POS);
        rightServo.setPosition(RIGHT_UNRETRACT_POS);
        //frontServo.setPosition(FRONT_UNRETRACT_POS);
    }
    public void loop(){
        if (ledTimer.time(TimeUnit.MILLISECONDS) < 450){
            slideDisplay = true;
        } else {
            slideDisplay = false;
        }
        linearSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller.setPID(p, i, d);
        int slidePos = linearSlides.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        f = h * linearSlides.getCurrentPosition() + b;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;
        if (!zeroMode) {
            linearSlides.setPower(power);
        }
/*        if (gripper.getPosition() == 0.75 && !slideDisplay) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        } else if (!slideDisplay){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
        } else {
            if (target == equate(LINEAR_SLIDES_NORM) || target == equate(LINEAR_SLIDES_IN_CONE)){
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
            } else if (target == equate(LINEAR_SLIDES_LOW)){
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (target == equate(LINEAR_SLIDES_MEDIUM)){
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
            } else if (target == equate(LINEAR_SLIDES_HIGH)){
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            } else if (!automation){
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
            }
        }*/

//        telemetry.addData("pos ", slidePos);
//        telemetry.addData("target ", target);
//        telemetry.addData("power ", power);
//        telemetry.addData("current ", linearSlides.getCurrent(CurrentUnit.MILLIAMPS));
//        telemetry.addData("slideDisplay ", slideDisplay);
//        telemetry.addData("LED Timer ", ledTimer);
//        telemetry.addData("Color Red", colorSensor1.red());
//        telemetry.addData("Color Green", colorSensor1.green());
//        telemetry.addData("Color Blue", colorSensor1.blue());
        telemetry.update();
    }

}