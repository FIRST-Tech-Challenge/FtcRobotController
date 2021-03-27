package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.EncodedMotorGroup;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.motor.MotorGroup;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.teamcode.util.Encoder;
import com.technototes.library.hardware.sensor.IMU;

/** Class for the hardware devices of the robot
 *
 */
public class Hardware implements Loggable {

    //drivebase
    public EncodedMotor<DcMotorEx> flDriveMotor;
    public EncodedMotor<DcMotorEx> frDriveMotor;
    public EncodedMotor<DcMotorEx> rlDriveMotor;
    public EncodedMotor<DcMotorEx> rrDriveMotor;

    public Encoder leftOdometryEncoder;
    public Encoder rightOdometryEncoder;
    public Encoder frontOdometryEncoder;

    public IMU imu;

    //index
    public Servo indexArmServo;
    public Servo indexPivotServo;

    //intake
    public Motor intakeMotor1;
    public Motor intakeMotor2;
    public MotorGroup intakeMotorGroup;

    //shooter
    public EncodedMotor<DcMotor> shooterMotor1;
    public EncodedMotor<DcMotor> shooterMotor2;
    public EncodedMotorGroup shooterMotorGroup;

    public Servo shooterFlapServo;


    //wobble
    public Servo wobbleArmServo;
    public Servo wobbleClawServo;

    public Hardware(){
        flDriveMotor = new EncodedMotor<>("flMotor");
        frDriveMotor = new EncodedMotor<>("frMotor");
        rlDriveMotor = new EncodedMotor<>("rlMotor");
        rrDriveMotor = new EncodedMotor<>("rrMotor");

        leftOdometryEncoder = new Encoder("shooter2");
        rightOdometryEncoder = new Encoder("intake2");
        frontOdometryEncoder = new Encoder("intake1");

        imu = new IMU("imu");

        indexArmServo = new Servo("indexarm");
        indexPivotServo = new Servo("indexpivot");

        intakeMotor1 = new Motor<>("intake1");
        intakeMotor2 = new Motor<>("intake2");
        //TODO fix this warning
        intakeMotorGroup = new MotorGroup(intakeMotor1, intakeMotor2);

        shooterMotor1 = new EncodedMotor<>("shooter1");
        shooterMotor2 = new EncodedMotor<>("shooter2");
        shooterMotorGroup = new EncodedMotorGroup(shooterMotor1.invert(), shooterMotor2.invert());

        shooterFlapServo = new Servo("flapservo");

        wobbleArmServo = new Servo("wobblearm");
        wobbleClawServo = new Servo("wobbleclaw");
    }
}
