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
public class Intake {
    private PIDController controller;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    Init init;

    Servo slideServo1;
    Servo slideServo2;
    Servo stringServo;
    DcMotor wheelMotor;

    public Intake(Init init){
        this.init = init;
    }
}
