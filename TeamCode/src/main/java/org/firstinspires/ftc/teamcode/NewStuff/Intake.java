package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private final OpModeUtilities opModeUtilities;
    public DcMotor wheelMotor;

    public DcMotor stackPusher;

    public IntakeState state;

    public static final double P_CONSTANT = 0.01;

    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.state = new IntakeState();
        setUpHardware();
    }

    private void setUpHardware() {
        wheelMotor = opModeUtilities.getHardwareMap().dcMotor.get("intake");
        stackPusher = opModeUtilities.getHardwareMap().dcMotor.get("stackAttachment");

        wheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stackPusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stackPusher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double intakeToTicksCalcPowerParallelSequence(GenericState conditionState) {
        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {
                return P_CONSTANT * state.getError();
            }
        }

        return 0;
    }

    public void setState(double currentTicks, double currentTargetTicks) {
        this.state.setCurrentTargetTicks(currentTargetTicks);
        this.state.setCurrentTicks(currentTicks);
    }

    public void update(GenericState conditionState) {
        if (!this.state.isDone()) {
            intakeToTicksCalcPowerParallelSequence(conditionState);
        }
    }

}