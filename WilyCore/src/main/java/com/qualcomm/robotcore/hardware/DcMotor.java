package com.qualcomm.robotcore.hardware;

public interface DcMotor extends DcMotorSimple {
    DcMotorController getController();
    int getPortNumber();

    enum ZeroPowerBehavior
    {
        UNKNOWN,
        BRAKE,
        FLOAT
    }

    void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior);
    ZeroPowerBehavior getZeroPowerBehavior();
    boolean getPowerFloat();
    void setTargetPosition(int position);
    int getTargetPosition();
    boolean isBusy();
    int getCurrentPosition();
    enum RunMode
    {
        RUN_WITHOUT_ENCODER,
        RUN_USING_ENCODER,
        RUN_TO_POSITION,
        STOP_AND_RESET_ENCODER;

        public boolean isPIDMode() {
            return this==RUN_USING_ENCODER || this==RUN_TO_POSITION;
        }
    }

    void setMode(RunMode mode);
    RunMode getMode();

}
