package org.firstinspires.ftc.teamcode.Systems.Mechaisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.AnalogServo;

public class Bucket {

    private DcMotorEx rollerMotor;
    private AnalogServo bucketServo;
    private Servo gateServo;

    public enum BucketState {
        bucketDown,
        bucketUp;
    }

    public Bucket (Hardware hardware) {
        rollerMotor = hardware.intakeRoller;
        bucketServo = new AnalogServo(hardware.intakePivot, hardware.intakePivotEnc);
        gateServo = hardware.intakeDoor;
    }

    public void setBucketPosition(double position) {
        bucketServo.setPos(position);
    }

    public void setGatePosition(double position) {
        gateServo.setPosition(position);
    }

    public void setRollerPower(double power) {
        rollerMotor.setPower(power);
    }



}
