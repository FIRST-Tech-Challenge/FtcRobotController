package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class tater {
    public static DcMotor frontLeft = null;
    public static DcMotor frontRight = null;
    public static DcMotor backleft = null;
    public static DcMotor backright = null;

    public static BNO055IMU imu;


    static HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public tater() {
    }
    public static void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
    }
}
