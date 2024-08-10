package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private final OpModeUtilities opModeUtilities;
    public DcMotor wheelMotor;

    public Servo stackPusher;

    public static final double P_CONSTANT = 0.004;


    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {
        wheelMotor = opModeUtilities.getHardwareMap().dcMotor.get("intake");
        stackPusher = opModeUtilities.getHardwareMap().servo.get("stackAttachment");

        wheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*

    public double intakeToTicksCalcPowerParallelSequence(GenericState conditionState) {
        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {
                return P_CONSTANT * state.getError();
            }

            return 0;
        }

        return 0;
    }

    public void setState(double currentTicks, double currentTargetTicks) {
        this.state.setCurrentTargetTicks(currentTargetTicks);
        this.state.setCurrentTicks(currentTicks);
    }

    public void update(GenericState conditionState) {
        if (!this.state.isDone() && conditionState.isDone()) {
            wheelMotor.setPower(intakeToTicksCalcPowerParallelSequence(conditionState));
        }
    }

     */

}