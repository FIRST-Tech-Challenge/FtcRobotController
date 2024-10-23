package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    Init init;

    Servo slideServo1;
    Servo slideServo2;
    Servo stringServo;
    DcMotor wheelMotor;

    public Intake(Init init){
        this.init = init;

        slideServo1 = init.getSlideServo1();
        slideServo2 = init.getSlideServo2();
        stringServo = init.getStringServo();
        wheelMotor = init.getWheelMotor();
    }
}
