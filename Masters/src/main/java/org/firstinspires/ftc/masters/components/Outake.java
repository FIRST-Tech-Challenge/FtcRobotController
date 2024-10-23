package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outake implements Component{

    private PIDController controller;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public int target = 0;

    private final double ticks_in_degree = 1425 / 180.0;

    public static double open = 0.7;
    public static double close = 0.45;
    public static double wristF = 0.9;
    public static double wristB = 0.7;

    private final DcMotor extendSlide;
    private final DcMotor rotateSlide;
    private final Servo claw;
    private final Servo clawWrist;

    Telemetry telemetry;
    Init init;

    public Outake(Init init, Telemetry telemetry){

        this.init=init;
        this.telemetry=telemetry;
        this.extendSlide=init.getExtendSlide();
        this.rotateSlide=init.getRotateSlide();
        this.claw=init.getClaw();
        this.clawWrist=init.getClawWrist();
        initializeHardware();

    }

    public void initializeHardware() {

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        target = 0;

    }

    public void slidesMove(int target) {

        int rotatePos = rotateSlide.getCurrentPosition();
        double pid = controller.calculate(rotatePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double lift = pid + ff;

        rotateSlide.setPower(lift);

    }

    public void extend() {
        extendSlide.setPower(1);
    }

    public void extendPower(double power) {
        extendSlide.setPower(power);
        telemetry.addData("Pos", extendSlide.getCurrentPosition());
        telemetry.update();
    }
    public void extendPos(int pos) {
        extendSlide.setTargetPosition(pos);
        extendSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendSlide.setPower(.7);
    }

    public void retract() {
        extendSlide.setPower(-1);
    }
    public void rotateStop() {
        rotateSlide.setPower(0);
    }

    public void extendStop() {
        extendSlide.setPower(0);
    }

    public void open(){ claw.setPosition(open); }
    public void close(){ claw.setPosition(close); }

    public void forward(){ clawWrist.setPosition(wristF); }
    public void backward(){ clawWrist.setPosition(wristB); }

    public void rotateUp(){ rotateSlide.setPower(1); }
    public void rotateDown(){ rotateSlide.setPower(-1); }

    public DcMotor getExtendSlide(){
        return extendSlide;
    }

    public DcMotor getRotateSlide(){
        return rotateSlide;
    }

}
