package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slide {
    public static double P = 0.046;
    public static double I = 0.000002;
    public static double D = 0.0855;
    double error = 0;
    double derivative = 0;
    double integralsum = 0;
    double lasterror = 0;
    double slidesholdpower = 0;
    boolean stopPID = false;
    boolean resettimer = true;
    private DcMotor slides;
    ElapsedTime timer = new ElapsedTime();
    public void slidesinit(HardwareMap hardwareMap) {
        slides = hardwareMap.dcMotor.get("slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.reset();
    }
    public void resettimer() {
        timer.reset();
    }
    public void stopPID() {
        stopPID = true;
        slides.setPower(0);
    }
    public void slidego(double targetposition) {
            double dt = timer.seconds();
            timer.reset();
            error = targetposition - slides.getCurrentPosition();
            derivative = (error - lasterror) / dt;
            integralsum = integralsum + (error * dt);
            slidesholdpower = (P * error) + (I * integralsum) + (D * derivative);
            slides.setPower(slidesholdpower);
            lasterror = error;

    }
}
