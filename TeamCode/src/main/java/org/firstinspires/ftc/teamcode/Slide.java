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
    private DcMotor slides;
    ElapsedTime timer = new ElapsedTime();
    public void slidesinit(HardwareMap hardwareMap) {
        DcMotor slides = hardwareMap.dcMotor.get("slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer = new ElapsedTime();
    }
    public void stopPID() {
        stopPID = true;
        slides.setPower(0);
    }
    public void slidego(double targetposition) {
        timer.reset();
        while (stopPID !=false) {
            error = targetposition - slides.getCurrentPosition();
            derivative = (error - lasterror) / timer.seconds();
            integralsum = integralsum + (error * timer.seconds());
            slidesholdpower = (P * error) + (I * integralsum) + (D * derivative);
            slides.setPower(slidesholdpower);
            lasterror = error;
        }
    }
}
