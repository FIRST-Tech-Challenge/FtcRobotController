package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

public class ExtenderSubsystem {

    private DcMotor extender;
    private TouchSensor limitSwitch;

    private int extenderZeroPosition;

    ElapsedTime runtime;
    Telemetry telemetry;

    public enum Position {
        EXTENDED,
        RETRACTED,
        HALFWAY
    }

    public ExtenderSubsystem(DcMotor extender, TouchSensor limitSwitch, ElapsedTime runtime, Telemetry telemetry) {
        this.extender = extender;
        this.limitSwitch = limitSwitch;

        this.extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extender.setDirection(DcMotorSimple.Direction.FORWARD);

        this.runtime = runtime;
        this.telemetry = telemetry;
        zeroPosition();
    }

    public void extendToPosition(Position extensionPosition) {
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
        extender.setTargetPosition(targetPosition);
    }

    public void extendToCustomPosition(double targetPositionInches) {
        int extenderMaxPosition = (int) (Constants.extenderMaxPositionInches * Constants.extenderMotorCPI);
        int targetPosition = Range.clip((int) (targetPositionInches * Constants.extenderMotorCPI), extenderZeroPosition, extenderMaxPosition);

        extender.setTargetPosition(targetPosition);
    }

    private void zeroPosition() {
        double startTime = runtime.seconds();
        while (!limitSwitch.isPressed()) {
            extender.setPower(Constants.extenderZeroingPower);

            if (runtime.seconds() - startTime > 8) {
                break;
            }
        }
        extender.setPower(0.0);
        extenderZeroPosition = (int) (extender.getCurrentPosition() + (1/8 * Constants.extenderMotorCPI));
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(extenderZeroPosition);
    }

    public int getExtenderPosition() {
        return extender.getCurrentPosition();
    }
}
