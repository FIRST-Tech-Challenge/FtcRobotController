package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.EncodedMotorGroup;
import com.technototes.library.hardware.motor.Motor;
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
    public Motor<DcMotor> indexMotor;

    //intake
    public Motor<DcMotor> intakeMotor;

    //shooter
    public EncodedMotor<DcMotor> shooterMotor1;
    public EncodedMotor<DcMotor> shooterMotor2;

    //wobble
    public Servo wobbleClawServo;
    public Servo wobbleArmServo;

    public Hardware(){

        flDriveMotor = new Motor<>("flMotor");
        frDriveMotor = new Motor<>("frMotor");
        rlDriveMotor = new Motor<>("rlMotor");
        rrDriveMotor = new Motor<>("rrMotor");

        imu = new IMU("imu");

        indexMotor = new Motor<>("index");

        intakeMotor = new Motor<>("intake");

        shooterMotor1 = new EncodedMotor<>("shooter1");
        shooterMotor2 = new EncodedMotor<>("shooter2");

        wobbleArmServo = new Servo("arm");
        wobbleClawServo = new Servo("claw");
    }
}
