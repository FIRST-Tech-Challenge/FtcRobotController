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

public class NewHardwareMap {
    // pinpoint

    private GoBildaPinpointDriver odo;

    // Drivetrain
    private final DcMotor frontRightMotor;
    private final DcMotor frontLeftMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;


    // shooter
    private final MotorEx flywheelMotor;
    private final MotorEx turretMotor;
    private final CRServo pitchServo;
    private final AnalogInput pitchEncoder;
    private final Limelight3A limelight;

    //intake
    private final MotorEx intakeMotor;

    // Transfer
    private final Servo transferServo;
    private final AnalogInput transferEncoder;

    private VoltageSensor voltageSensor;


    public NewHardwareMap(HardwareMap hardwareMap) {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");

        flywheelMotor = new MotorEx(hardwareMap, "FM", Motor.GoBILDA.BARE);
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        turretMotor = new MotorEx(hardwareMap, "TM", Motor.GoBILDA.RPM_1150); // TODO: get right RPM
        pitchServo = new CRServo(hardwareMap, "PS");
        pitchEncoder = hardwareMap.get(AnalogInput.class, "PE");
        transferEncoder = hardwareMap.get(AnalogInput.class, "TE");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intakeMotor = new MotorEx(hardwareMap, "IM", Motor.GoBILDA.RPM_1150);
        transferServo = hardwareMap.get(Servo.class, "TS");


        voltageSensor = hardwareMap.voltageSensor.iterator().next();
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

    public Limelight3A getLimelight() {
        return limelight;
    }

    public MotorEx getFlywheelMotor() {
        return flywheelMotor;
    }

    public MotorEx getTurretMotor() {
        return turretMotor;
    }

    public CRServo getPitchServo() {
        return pitchServo;
    }

    public AnalogInput getPitchEncoder() {
        return pitchEncoder;
    }

    public MotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public Servo getTransferServo() {
        return transferServo;
    }


    public AnalogInput getTransferEncoder() {
        return transferEncoder;
    }


    public VoltageSensor getVoltageSensor() {
        return voltageSensor;
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
    }

}