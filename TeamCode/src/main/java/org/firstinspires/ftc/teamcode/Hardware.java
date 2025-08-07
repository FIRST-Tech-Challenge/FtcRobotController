package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    // Motors
    public static DcMotor lift1;
    public static DcMotor lift2;
    public static DcMotor lift3;

    public static DcMotor lf;
    public static DcMotor rf;
    public static DcMotor lb;
    public static DcMotor rb;

    // Servos
    public static Servo claw;
    public static Servo larm;
    public static Servo rarm;

    public static Servo llinkage;
    public static Servo rlinkage;

    public static void init(HardwareMap hwMap) {
        lift1 = hwMap.get(DcMotor.class, Specifications.EXTENSION_MOTOR_MAIN);
        lift2 = hwMap.get(DcMotor.class, Specifications.EXTENSION_MOTOR_AUX1);
        lift3 = hwMap.get(DcMotor.class, Specifications.EXTENSION_MOTOR_AUX2);

        lf = hwMap.get(DcMotor.class, Specifications.FTLF_MOTOR);
        rf = hwMap.get(DcMotor.class, Specifications.FTRT_MOTOR);
        lb = hwMap.get(DcMotor.class, Specifications.BKLF_MOTOR);
        rb = hwMap.get(DcMotor.class, Specifications.BKRT_MOTOR);

        claw = hwMap.get(Servo.class, Specifications.CLAW_SERVO);
        larm = hwMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
        rarm = hwMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);

        llinkage = hwMap.get(Servo.class, Specifications.EXTENSION_ARM_LEFT);
        rlinkage = hwMap.get(Servo.class, Specifications.EXTENSION_ARM_RIGHT);
    }
}
