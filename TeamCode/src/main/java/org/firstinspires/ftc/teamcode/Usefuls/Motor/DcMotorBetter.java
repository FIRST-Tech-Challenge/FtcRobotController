package org.firstinspires.ftc.teamcode.Usefuls.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;

public class DcMotorBetter implements Motor {
    private DcMotorEx dcMotorEx;
    private DcMotorEx.Direction direction = DcMotorEx.Direction.FORWARD;
    private DcMotorEx.RunMode runMode = DcMotorEx.RunMode.RUN_TO_POSITION;
    private DcMotorEx.ZeroPowerBehavior zeroPowerBehavior = DcMotorEx.ZeroPowerBehavior.FLOAT;
    private double lowerBound = 0.0;
    private double upperBound = 1.0;
    private double position = 0.0;
    private double power = 0.0;
    private double lastPower = 0.0;

    public DcMotorBetter(DcMotorEx dcMotorEx) {
        this.dcMotorEx = dcMotorEx;
    }

    public DcMotorBetter setLowerBound(double bound) {
        this.lowerBound = bound;
        return this;
    }
    public DcMotorBetter setUpperBound(double bound) {
        this.upperBound = bound;
        return this;
    }

    public DcMotorBetter setDirection(DcMotorEx.Direction direction) {
        this.direction = direction;
        return this;
    }
    public DcMotorBetter setMode(DcMotorEx.RunMode runMode) { this.runMode = runMode; return this; }

    public DcMotorBetter setPosition(double position) {
        this.position = position;
        this.runMode = DcMotorEx.RunMode.RUN_TO_POSITION;
        return this;
    }

    public DcMotorBetter setPower(double power) {
        this.power = power;
        this.runMode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        return this;
    }

    public DcMotorBetter setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
        return this;
    }

    public DcMotorBetter addPosition(double position) {
        this.position += position;
        this.runMode = DcMotorEx.RunMode.RUN_TO_POSITION;
        return this;
    }

    public DcMotorBetter addPower(double power) {
        this.power += power;
        this.runMode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        return this;
    }

    public double getPower() {
        return this.power;
    }

    public double getTargetPosition() {
        return M.normalize(this.dcMotorEx.getTargetPosition(), this.lowerBound, this.upperBound);
    }

    public double getCurrentPosition() {
        return M.normalize(this.dcMotorEx.getCurrentPosition(), this.lowerBound, this.upperBound);
    }
    public double getCurrentPositionRAW(){
        return this.dcMotorEx.getCurrentPosition();
    }

    public double getCurrentAMPS(){
        return this.dcMotorEx.getCurrent(CurrentUnit.AMPS);
    }

    public DcMotorEx.RunMode getMode() {
        return this.dcMotorEx.getMode();
    }

    public DcMotorBetter stop() {
        this.position = this.getCurrentPosition();
        this.power = 0.0;
        return this;
    }

    public DcMotorBetter stopAndResetEncoder() {
        this.stop();
        this.dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return this;
    }

    public boolean isBusy() {
        return this.dcMotorEx.isBusy();
    }

    public void resetEncoder() {
        this.dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update() {
        switch (this.runMode) {
            case RUN_TO_POSITION: {
                double position = M.lerp(this.lowerBound, this.upperBound, this.position);
                if (this.direction != this.dcMotorEx.getDirection()) position = this.upperBound - (position - this.lowerBound);
                this.dcMotorEx.setTargetPosition((int) position);
                this.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                break;
            }
            case RUN_WITHOUT_ENCODER: {
                double power = this.power;
                if (this.direction != this.dcMotorEx.getDirection()) power = -power;
                this.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                if(Math.abs(this.power)<.3){
                    this.dcMotorEx.setPower(0.0);
                }
                if(Math.abs(this.power - this.lastPower) > .05){
                    this.dcMotorEx.setPower(power);
                }
//                this.power = 0.0;
                this.position = this.getCurrentPosition();
                this.lastPower = this.power;
                break;
            }
        }
    }
}