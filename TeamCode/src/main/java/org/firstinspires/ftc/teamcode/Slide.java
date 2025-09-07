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
    public static double P = 0.036;
    public static double D = 0;

    double error = 0;
    double derivative = 0;
    double lasterror = 0;
    double slidesholdpower = 0;
    private DcMotor slides;
    ElapsedTime timer = new ElapsedTime();

    public void resettimer() {
        timer.reset();
    }
    public void slidego(double targetposition, HardwareMap hardwareMap) {
            slides = hardwareMap.dcMotor.get("slides");
            double dt = timer.seconds();
            if(dt < 0.01) {
                dt = 0.01;
            }
            timer.reset();
            error = targetposition - slides.getCurrentPosition();
            slidesholdpower = (P * error);
            if(slidesholdpower < 0) {
                slides.setPower(Math.max(slidesholdpower,-1));
            } else if (slidesholdpower > 0) {
                slides.setPower(Math.min(slidesholdpower,1));
            }
            lasterror = error;

    }

}
