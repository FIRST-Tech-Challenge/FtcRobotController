package org.firstinspires.ftc.teamcode.Mechanism;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

import java.util.concurrent.TimeUnit;

@Config
public class LinearSlides extends HWMap {

    public static final double ticksPerRotation = 384.5;// 435 RPM motor 5202 GoBilda TPR
    private static final double inFactor_MM = 125;
    public static double diameterOfSpool = 34;

    public static double target = 0;
    private final double ticks_in_degrees = 384.5/360;
    public static boolean zeroMode = false;
    private RetractOdo retractOdo = new RetractOdo(telemetry, hardwareMap);
    private ElapsedTime timer = new ElapsedTime();
    private Blinkin blinkin = new Blinkin(telemetry, hardwareMap);
    public static ElapsedTime ledTimer = new ElapsedTime();

    public static boolean automation = false; //sus - need FBI

    public enum Ls {
        LOW(372.5),
        MEDIUM(632.5),
        HIGH(901),
        NORM(105),
        IN_CONE(0);

        public final double level;

        private Ls(double level){
            this.level = equate(level);
        }
    }

    public enum Stack {
        ABOVE_CONE_1(133.35),
        ABOVE_CONE_2(165.1),
        ABOVE_CONE_3(196.85),
        ABOVE_CONE_4(228.6),
        ABOVE_CONE_5(248),
        INSIDE_CONE_1(ABOVE_CONE_1.level - inFactor_MM),
        INSIDE_CONE_2(ABOVE_CONE_2.level - inFactor_MM),
        INSIDE_CONE_3(ABOVE_CONE_3.level - inFactor_MM),
        INSIDE_CONE_4(ABOVE_CONE_4.level - inFactor_MM),
        INSIDE_CONE_5(ABOVE_CONE_5.level - inFactor_MM);

        public final double level;

        private Stack(double level){
            this.level = equate(level);
        }
    }


    //SLIDE PID____________________________________________________________________________________
    public static double p = 0.008, i = 0, d = 0.00012 ;
    public double f;
    public static double h = 0.000025, b = 0.01;
    private PIDController SlidePIDController = new PIDController(p, i, d);


    public LinearSlides(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
    }

    public void init() {
        setGripperPosition(1.0);
        setHeight(0);
        retractOdo.retractOdometryServos();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        zeroMode = false;
    }

    public void setGripperPosition(double position) {
        gripper.setPosition(position);
    }

    public static int equate(double height) {
        return (int)(ticksPerRotation * (height / (diameterOfSpool * Math.PI)));
    }

    public static void setHeight(double level) {
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
                target = Stack.ABOVE_CONE_5.level;
                automation = false;
            } else if (automation){
                target = 0;
            }
    }

    public void loop(){
        if (ledTimer.time(TimeUnit.MILLISECONDS) < 450){
            blinkin.slideDisplay = true;
        } else {
            blinkin.slideDisplay = false;
        }
        linearSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlidePIDController.setPID(p, i, d);
        int slidePos = linearSlides.getCurrentPosition();
        double pid = SlidePIDController.calculate(slidePos, target);
        f = h * linearSlides.getCurrentPosition() + b;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;
        if (!zeroMode) {
            linearSlides.setPower(power);
        }

        telemetry.update();
    }

}