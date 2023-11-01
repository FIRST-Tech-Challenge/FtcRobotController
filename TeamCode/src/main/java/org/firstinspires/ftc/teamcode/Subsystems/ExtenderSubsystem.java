package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MiniPID;

public class ExtenderSubsystem {
    private DcMotor extenderMotor;
    private TouchSensor limitSwitch;

    private double extenderMotorPower;
    private int extenderZeroPosition;

    public ExtenderSubsystem(DcMotor extenderMotor, TouchSensor limitSwitch) {
        this.extenderMotor = extenderMotor;
        this.limitSwitch = limitSwitch;

        this.extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extenderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        zeroPosition();
    }

    public void extendToPosition(ExtensionPosition extensionPosition) {
        int targetPosition;

        switch (extensionPosition) {
            case EXTENDED:
                targetPosition = (int) (Constants.extenderMaxPositionInches * Constants.extenderMotorCPI);
                break;
            case HALFWAY:
                targetPosition = (int) (Constants.extenderMaxPositionInches/2 * Constants.extenderMotorCPI);
                break;
            case RETRACTED:
                targetPosition = extenderZeroPosition;
                break;
            default:
                return;
        }
        extenderMotor.setTargetPosition(targetPosition);
    }

    public void extendToCustomPosition(double targetPositionInches) {
        int extenderMaxPosition = (int) (Constants.extenderMaxPositionInches * Constants.extenderMotorCPI);
        int targetPosition = Range.clip((int) (targetPositionInches * Constants.extenderMotorCPI), extenderZeroPosition, extenderMaxPosition);

        extenderMotor.setTargetPosition(targetPosition);
    }

    private void zeroPosition() {
        while (!limitSwitch.isPressed()) {
            extenderMotor.setPower(Constants.extenderZeroingPower);
        }
        extenderMotor.setPower(0.0);
        extenderZeroPosition = (int) (extenderMotor.getCurrentPosition() + (1/8 * Constants.extenderMotorCPI));
        extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extenderMotor.setTargetPosition(extenderZeroPosition);
    }

    public enum ExtensionPosition {
        EXTENDED,
        RETRACTED,
        HALFWAY
    }
}
