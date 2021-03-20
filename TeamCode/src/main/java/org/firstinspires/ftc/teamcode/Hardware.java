package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.EncodedMotorGroup;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.motor.MotorGroup;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.logger.Loggable;

/** Class for the hardware devices of the robot
 *
 */
public class Hardware implements Loggable {

    //drivebase
    public Motor<DcMotor> flDriveMotor;
    public Motor<DcMotor> frDriveMotor;
    public Motor<DcMotor> rlDriveMotor;
    public Motor<DcMotor> rrDriveMotor;

    public IMU imu;

    //index
    public Servo indexArmServo;
    public Servo indexPivotServo;

    //intake
    public Motor<DcMotor> intakeMotor1;
    public Motor<DcMotor> intakeMotor2;
    public MotorGroup intakeMotorGroup;

    //shooter
    public EncodedMotor<DcMotor> shooterMotor1;
    public EncodedMotor<DcMotor> shooterMotor2;

    //wobble
    public Servo wobbleServo1;
    public Servo wobbleServo2;

    public Hardware(){
        //TODO .invert instead of setInverted(true);
        flDriveMotor = new Motor<>("flMotor");
        frDriveMotor = new Motor<>("frMotor");
        rlDriveMotor = new Motor<>("rlMotor");
        rrDriveMotor = new Motor<>("rrMotor");

        imu = new IMU("imu");

        indexArmServo = new Servo("indexarm");
        indexPivotServo = new Servo("indexpivot");

        intakeMotor1 = new Motor<>("intake1");
        intakeMotor2 = new Motor<>("intake2");
        intakeMotorGroup = new MotorGroup(intakeMotor1, intakeMotor2);

        shooterMotor1 = new EncodedMotor<>("shooter1");
        shooterMotor2 = new EncodedMotor<>("shooter2");

        wobbleServo1 = new Servo("wobble1");
        wobbleServo2 = new Servo("wobble2");
    }
}
