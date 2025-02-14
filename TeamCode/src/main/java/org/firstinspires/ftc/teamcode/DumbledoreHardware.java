package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.AutoClearEncoder;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.EncoderFor;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.MotorSet;
import org.firstinspires.ftc.teamcode.hardware.Reversed;
import org.firstinspires.ftc.teamcode.hardware.ZeroPower;
import org.firstinspires.ftc.teamcode.mmooover.TriOdoProvider;


public class DumbledoreHardware extends HardwareMapper {
    // left = left motor = exp 0 frontLeft
    // right = right motor = ctr 0 frontRight
    // center = ctr 3 intake

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontLeft;

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontRight;

    @HardwareName("backLeft")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @Reversed
    public DcMotor backLeft;

    @HardwareName("backRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backRight;

    @EncoderFor("frontLeft")
    @AutoClearEncoder
    public Encoder encoderLeft;



    @EncoderFor("frontRight")
    @AutoClearEncoder
    public Encoder encoderRight;

   /* @HardwareName("gyro")
    public NavxMicroNavigationSensor gyro;

    @Override
    public Encoder getLeftEncoder() {
        return encoderLeft;
    }
 @EncoderFor("intake")
    @AutoClearEncoder
    public Encoder encoderCenter;

    @Override
    public Encoder getRightEncoder() {
        return encoderRight;
    }

    @Override
    public Encoder getCenterEncoder() {
        return encoderCenter;
    }

    @Override
    public double getTrackWidth() {
        return 14 + 7 / 16.;
    }

    @Override
    public double getForwardOffset() {
        return -(6 + 3 / 4.);
    }

    @Override
    public int getEncoderTicksPerRevolution() {
        return 8192;
    }

    @Override
    public double getEncoderWheelRadius() {
        return 0.70;
    }
*/
    public MotorSet driveMotors;

    public DumbledoreHardware(HardwareMap hwMap) {
        super(hwMap);
        driveMotors = new MotorSet(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );
    }

}
