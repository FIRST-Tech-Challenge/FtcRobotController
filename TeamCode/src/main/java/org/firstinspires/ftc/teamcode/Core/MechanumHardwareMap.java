package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class MechanumHardwareMap {
    // pinpoint

    private GoBildaPinpointDriver odo;

    // Drivetrain
    private final DcMotor frontRightMotor;
    private final DcMotor frontLeftMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;


    public MechanumHardwareMap(HardwareMap hardwareMap) {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }

    public DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }

    public DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }

    public DcMotor getBackRightMotor() {
        return backRightMotor;
    }

    public DcMotor getBackLeftMotor() {
        return backLeftMotor;
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
    }

}