package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class hwMap {
    // shooter
    private final MotorEx flywheelMotor;
    private final MotorEx turretMotor;
    private final CRServo pitchServo;
    private final AnalogInput pitchEncoder;
    private final Limelight3A limelight;

    //intake
    private final MotorEx intakeMotor;

    // Transfer
    private final MotorEx transferMotor;
    private final Servo transferServo;
    private final AnalogInput transferEncoder;

    // Front Left Module Hardware
    public DcMotorEx FLM;
    public CRServoImplEx FLS;
    public AnalogInput FLE;

    // Front Right Module Hardware
    public DcMotorEx FRM;
    public CRServoImplEx FRS;
    public AnalogInput FRE;

    // Back Right Module Hardware
    public DcMotorEx BRM;
    public CRServoImplEx BRS;
    public AnalogInput BRE;

    // Back Left Module Hardware
    public DcMotorEx BLM;
    public CRServoImplEx BLS;
    public AnalogInput BLE;

    private VoltageSensor voltageSensor;


    public hwMap (HardwareMap hardwareMap) {
        flywheelMotor = new MotorEx(hardwareMap,"FM", Motor.GoBILDA.BARE);
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        turretMotor = new MotorEx(hardwareMap,"TM", Motor.GoBILDA.RPM_1150); // TODO: get right RPM
        pitchServo = new CRServo(hardwareMap, "PS");
        pitchEncoder = hardwareMap.get(AnalogInput.class, "PE");
        transferEncoder = hardwareMap.get(AnalogInput.class, "TE");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intakeMotor = new MotorEx(hardwareMap, "IM", Motor.GoBILDA.RPM_1150);
        transferMotor = new MotorEx(hardwareMap, "TRM", Motor.GoBILDA.RPM_312);
        transferServo = hardwareMap.get(Servo.class, "TS");

        FLM = hardwareMap.get(DcMotorEx.class, "FLM");
        FRM = hardwareMap.get(DcMotorEx.class, "FRM");
        BLM = hardwareMap.get(DcMotorEx.class, "BLM");
        BRM = hardwareMap.get(DcMotorEx.class, "BRM");

        FLS = hardwareMap.get(CRServoImplEx.class, "FLS");
        FRS = hardwareMap.get(CRServoImplEx.class, "FRS");
        BLS = hardwareMap.get(CRServoImplEx.class, "BLS");
        BRS = hardwareMap.get(CRServoImplEx.class, "BRS");

        FLE = hardwareMap.get(AnalogInput.class, "FLE");
        FRE = hardwareMap.get(AnalogInput.class, "FRE");
        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BRE = hardwareMap.get(AnalogInput.class, "BRE");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
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

    public MotorEx getTransferMotor() {
        return transferMotor;
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
}