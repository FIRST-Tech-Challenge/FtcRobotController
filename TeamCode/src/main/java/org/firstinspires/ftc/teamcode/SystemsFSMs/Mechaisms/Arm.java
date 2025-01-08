package org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.AnalogServo;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.Hardware.Util.PosChecker;

public class Arm {

    private Logger logger;

    private AnalogServo rightServo;
    private AnalogServo leftServo;

    public enum Status {
        TransferPos,
        SpecIntakePos,
        SpecDepositPos,
        SampleDepositPos,
        Intermediate;
    }

    private double rightServoTargetPosition;
    private double leftServoTargetPosition;

    private double rightEncPosition;
    private double leftEncPosition;

    private boolean safeSlideDown;

    private Status status;

    public Arm(Hardware hardware, Logger logger) {
        this.logger = logger;

        rightServo = new AnalogServo(hardware.armRight, hardware.armRightEnc);
        leftServo = new AnalogServo(hardware.armLeft, hardware.armLeftEnc);
    }

    public void update() {
        rightEncPosition = rightServo.getPos();
        leftEncPosition = leftServo.getPos();

        findState();

        updateSlideSafe();
    }

    public void command() {
        rightServo.setPos(rightServoTargetPosition);
        leftServo.setPos(leftServoTargetPosition);
    }

    public void log() {
        logger.log("<b>" + "Arm" + "</b>", "", Logger.LogLevels.production);

        logger.log("Status", status, Logger.LogLevels.debug);
        logger.log("Slides Retract Safe", safeSlideDown, Logger.LogLevels.debug);

        logger.log("Right Target Pos", rightServoTargetPosition, Logger.LogLevels.developer);
        logger.log("Left Target Pos", leftServoTargetPosition, Logger.LogLevels.developer);
        logger.log("Right Encoder Pos", rightEncPosition, Logger.LogLevels.developer);
        logger.log("Left Encoder Pos", leftEncPosition, Logger.LogLevels.developer);
    }

    public Status getStatus() {
        return status;
    }

    public void setPosition(double position) {
        rightServoTargetPosition = position;
        leftServoTargetPosition = 1 - position;
    }

    public double getRightSetPosition() {
        return rightServoTargetPosition;
    }

    public boolean getSlideSafeDown() {
        return safeSlideDown;
    }

    private void findState() {

        if (PosChecker.atAngularPos(rightEncPosition, DepositConstants.armRightEncTransferPos, DepositConstants.armRightPositionTolerance) && rightServoTargetPosition == DepositConstants.armRightTransferPos) {
            status = Status.TransferPos;
        } else if (PosChecker.atAngularPos(rightEncPosition, DepositConstants.armRightEncSpecIntakePos, DepositConstants.armRightPositionTolerance) && rightServoTargetPosition == DepositConstants.armRightSpecIntakePos) {
            status = Status.SpecIntakePos;
        } else if (PosChecker.atAngularPos(rightEncPosition, DepositConstants.armRightEncSpecDepositPos, DepositConstants.armRightPositionTolerance) && rightServoTargetPosition == DepositConstants.armRightSpecDepositPos) {
            status = Status.SpecDepositPos;
        } else if (PosChecker.atAngularPos(rightEncPosition, DepositConstants.armRightEncSampleDepositPos, DepositConstants.armRightPositionTolerance) && rightServoTargetPosition == DepositConstants.armRightSampleDepositPos) {
            status = Status.SampleDepositPos;
        } else {
            status = Status.Intermediate;
        }

    }

    public void updateSlideSafe() {

        if (rightEncPosition >= DepositConstants.armRightEncSlideDownSafePos || (status == Status.TransferPos && rightServoTargetPosition == DepositConstants.armRightTransferPos)) {
            safeSlideDown = true;
        } else {
            safeSlideDown = false;
        }

    }

    public double getRightServoEncPos() {
        return rightEncPosition;
    }
}
