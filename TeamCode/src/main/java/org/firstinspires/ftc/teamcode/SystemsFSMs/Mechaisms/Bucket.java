package org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.AnalogServo;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.Hardware.Util.PosChecker;

public class Bucket {

    private DcMotorEx rollerMotor;
    private AnalogServo bucketServo;
    private Servo gateServo;
    private Logger logger;

    private double bucketTargetPos;
    private double bucketServoEncPosition;

    private double gateTargetPos;

    private double rollerPower;
    private double current = 0.00;

    public enum Status {
        up,
        down,
        intermediate;
    }

    private Status status;


    public Bucket (Hardware hardware, Logger logger) {
        this.logger = logger;

        rollerMotor = hardware.intakeRoller;
        bucketServo = new AnalogServo(hardware.intakePivot, hardware.intakePivotEnc);
        gateServo = hardware.intakeDoor;
    }

    public void update() {
        bucketServoEncPosition = bucketServo.getPos();
        current = rollerMotor.getCurrent(CurrentUnit.MILLIAMPS);
        findStatus();
    }

    public void command() {
        bucketServo.setPos(bucketTargetPos);
        gateServo.setPosition(gateTargetPos);
        rollerMotor.setPower(rollerPower);
    }

    public void log() {
        logger.log("<b>" + "Bucket" + "</b>", "", Logger.LogLevels.production);
        logger.log("Status", status, Logger.LogLevels.debug);

        logger.log("Bucket Servo Target Pos", bucketTargetPos, Logger.LogLevels.developer);
        logger.log("Bucket Servo Pos", bucketServoEncPosition, Logger.LogLevels.developer);
        logger.log("Gate Servo Target Pos", gateTargetPos, Logger.LogLevels.developer);
        logger.log("Roller Power", rollerPower, Logger.LogLevels.developer);
        logger.log("Roller Current", current, Logger.LogLevels.developer);
    }

    public void setBucketPosition(double position) {
        bucketTargetPos = position;
    }

    public void setGatePosition(double position) {
        gateTargetPos = position;
    }

    public void setRollerPower(double power) {
        rollerPower = power;
    }

    public Status getStatus() {
        return status;
    }

    public double getBucketServoEncPosition() {
        return bucketServoEncPosition;
    }

    private void findStatus() {
        if (PosChecker.atAngularPos(bucketServoEncPosition, IntakeConstants.bucketEncUpPosition, IntakeConstants.bucketEncPositionTolerance)) {
            status = Status.up;
        } else if (bucketServoEncPosition <= IntakeConstants.bucketEncDownPartialPosition) {
            status = Status.down;
        } else {
            status = Status.intermediate;
        }
    }
}
