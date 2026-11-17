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

public class NewIntakeTransferHWmap {
    // pinpoint

    private GoBildaPinpointDriver odo;

    // shooter
    private final MotorEx flywheelMotor;
/*    private final MotorEx turretMotor;
    private final CRServo pitchServo;
    private final AnalogInput pitchEncoder;*/

    //intake
    private final MotorEx intakeMotor;

    // Transfer
    private final MotorEx transferMotor;
    private final Servo transferServo;

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


    public NewIntakeTransferHWmap(HardwareMap hardwareMap) {
        flywheelMotor = new MotorEx(hardwareMap, "FM", Motor.GoBILDA.BARE);
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        // turretMotor = new MotorEx(hardwareMap, "TM", Motor.GoBILDA.RPM_1150); // TODO: get right RPM
        // pitchServo = new CRServo(hardwareMap, "PS");
        // pitchEncoder = hardwareMap.get(AnalogInput.class, "PE");

        intakeMotor = new MotorEx(hardwareMap, "IM", Motor.GoBILDA.RPM_1150);
        transferServo = hardwareMap.get(Servo.class, "TS");
        transferMotor = new MotorEx(hardwareMap, "TRM", Motor.GoBILDA.RPM_312);
    }

    public MotorEx getTransferMotor() {
        return transferMotor;
    }

    public MotorEx getFlywheelMotor() {
        return flywheelMotor;
    }

/*    public MotorEx getTurretMotor() {
        return turretMotor;
    }

    public CRServo getPitchServo() {
        return pitchServo;
    }

    public AnalogInput getPitchEncoder() {
        return pitchEncoder;
    }*/

    public MotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public Servo getTransferServo() {
        return transferServo;
    }
}