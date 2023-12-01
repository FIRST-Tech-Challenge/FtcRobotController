package org.firstinspires.ftc.teamcode.To_Be_Removed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MiniPID;

import java.awt.font.NumericShaper;

public class ErectorSubsystem {
    private DcMotor erectorHardwareMap;
    private double erectorPower;
    private DcMotor erector;

    private double erectorStartingPosition;

    private MiniPID erectPID;

    public ErectorSubsystem(DcMotor erectorHardwareMap) {
        this.erectorHardwareMap = erectorHardwareMap;
    }

    public void initialize() {
        erector = erectorHardwareMap;
        erector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        erector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        erectPID = new MiniPID(Constants.erectP, Constants.erectI, Constants.erectD);
        erectPID.setOutputLimits(-1.0, 1.0);

        erectorStartingPosition = erector.getCurrentPosition();
    }

    public void teleOPErect(Boolean gamepad2_DPadDown, Boolean gamepad2_DPadUp) {
        if (gamepad2_DPadUp && erector.getCurrentPosition() < Constants.maxErectorPositionInches * Constants.erectorMotorCPI) {
            erectorPower = Constants.teleOPErectorPower;
        }
        else if (gamepad2_DPadDown && erector.getCurrentPosition() > erectorStartingPosition) {
            erectorPower = -Constants.teleOPErectorPower;
        }
        else {
            erectorPower = 0.0;
        }

        erector.setPower(erectorPower);
    }

    public void autoErectInches(Direction direction, double amountInches) {
        double amountCounts = amountInches * Constants.erectorMotorCPI;

        boolean inRange = false;
        android.util.Range<Double> endRange = android.util.Range.create(-0.05, 0.05);
        double targetPosition;

        switch (direction) {
            case ERECT:
                while (inRange) {
                    targetPosition = erector.getCurrentPosition() + amountCounts;
                    Range.clip(targetPosition, erectorStartingPosition, Constants.maxErectorPositionInches * Constants.erectorMotorCPI);

                    erectorPower = erectPID.getOutput(erector.getCurrentPosition(), targetPosition);

                    if (endRange.contains(erectorPower)) {
                        inRange = true;
                    }

                    erector.setPower(erectorPower);
                }

                erector.setPower(0.0);
                break;

            case SHRINK:
                while (!inRange) {
                    targetPosition = erector.getCurrentPosition() - amountCounts;
                    Range.clip(targetPosition, erectorStartingPosition, Constants.maxErectorPositionInches * Constants.erectorMotorCPI);

                    erectorPower = erectPID.getOutput(erector.getCurrentPosition(), targetPosition);

                    if (endRange.contains(erectorPower)) {
                        inRange = true;
                    }

                    erector.setPower(erectorPower);
                }

                erector.setPower(0.0);
                break;

            default:
                break;
        }
    }

    public void autoErectPosition(Direction direction) {
        boolean inRange = false;
        android.util.Range<Double> endRange = android.util.Range.create(-0.05, 0.05);
        double targetPosition;

        switch (direction) {
            case ERECT:
                while (inRange) {
                    targetPosition = Constants.erectorErectPosition;
                    Range.clip(targetPosition, erectorStartingPosition, Constants.maxErectorPositionInches * Constants.erectorMotorCPI);

                    erectorPower = erectPID.getOutput(erector.getCurrentPosition(), targetPosition);

                    if (endRange.contains(erectorPower)) {
                        inRange = true;
                    }

                    erector.setPower(erectorPower);
                }

                erector.setPower(0.0);
                break;

            case HALFWAY:
                while (inRange) {
                    targetPosition = Constants.erectorHalfwayPosition;
                    Range.clip(targetPosition, erectorStartingPosition, Constants.maxErectorPositionInches * Constants.erectorMotorCPI);

                    erectorPower = erectPID.getOutput(erector.getCurrentPosition(), targetPosition);

                    if (endRange.contains(erectorPower)) {
                        inRange = true;
                    }

                    erector.setPower(erectorPower);
                }

                erector.setPower(0.0);
                break;

            case SHRINK:
                while (inRange) {
                    targetPosition = Constants.erectorShrunkPosition;
                    Range.clip(targetPosition, erectorStartingPosition, Constants.maxErectorPositionInches * Constants.erectorMotorCPI);

                    erectorPower = erectPID.getOutput(erector.getCurrentPosition(), targetPosition);

                    if (endRange.contains(erectorPower)) {
                        inRange = true;
                    }

                    erector.setPower(erectorPower);
                }

                erector.setPower(0.0);
                break;

            default:
                break;
        }
    }

    public enum Direction {
        ERECT,
        SHRINK,
        HALFWAY
    }
}
