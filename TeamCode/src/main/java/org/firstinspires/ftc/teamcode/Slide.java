package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

public class Slide {
    FtcDashboard dashboard;
    public static double P = 0.06;
    public static double I = 0.000015;
    public static double D = 0.0006;

    double error = 0;
    double derivative = 0;
    double integralsum = 0;
    double lasterror = 0;
    double slidesholdpower = 0;
    boolean stopPID = false;
    boolean resettimer = true;
    private DcMotor slides;
    ElapsedTime timer = new ElapsedTime();

    public void resettimer() {
        timer.reset();
    }
    public void stopPID() {
        stopPID = true;
        slides.setPower(0);
    }
    public void slidego(double targetposition, HardwareMap hardwareMap) {
            slides = hardwareMap.dcMotor.get("slides");
            double dt = timer.seconds();
            timer.reset();
            error = targetposition - slides.getCurrentPosition();
            derivative = (error - lasterror) / dt;
            integralsum = integralsum + (error * dt);
            slidesholdpower = (P * error) + (I * integralsum) + (D * derivative);

            slides.setPower(Math.min(slidesholdpower,1));
            lasterror = error;

    }

}
