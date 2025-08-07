package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    //singleton
    private static Hardware instance;

    // Motors
    public final DcMotor lift1;
    public final DcMotor lift2;
    public final DcMotor lift3;

    public final DcMotor lf;
    public final DcMotor rf;
    public final DcMotor lb;
    public final DcMotor rb;

    // Servos
    public final Servo claw;
    public final Servo lArm;
    public final Servo rArm;

    public final Servo lLinkage;
    public final Servo rLinkage;

    private Hardware(HardwareMap hwMap){
        this.lift1 = hwMap.get(DcMotor.class, Specifications.EXTENSION_MOTOR_MAIN);
        this.lift2 = hwMap.get(DcMotor.class, Specifications.EXTENSION_MOTOR_AUX1);
        this.lift3 = hwMap.get(DcMotor.class, Specifications.EXTENSION_MOTOR_AUX2);

        this.rf = hwMap.get(DcMotor.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotor.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotor.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotor.class, Specifications.BKRT_MOTOR); //rightback

        this.claw = hwMap.get(Servo.class, Specifications.CLAW_SERVO);
        this.lArm = hwMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
        this.rArm = hwMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);

        this.lLinkage = hwMap.get(Servo.class, Specifications.EXTENSION_ARM_LEFT);
        this.rLinkage = hwMap.get(Servo.class, Specifications.EXTENSION_ARM_RIGHT);
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }
}
