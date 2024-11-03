package org.firstinspires.ftc.teamcode.code2023;


import com.kuriosityrobotics.shuttle.hardware.LinearMotorControl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utilities.OpModeUtilities;

public class OuttakeSlide extends LinearMotorControl {
    private final DcMotorEx lsFront;
    private final DcMotorEx lsBack;

    private final double TICKS_PER_METER = 4754.31163987; //divide to get meters
    private OuttakeSlide(DcMotorEx lsBack, DcMotorEx lsFront) {
        this.lsBack = lsBack;
        this.lsFront = lsFront;
    }
    public OuttakeSlide(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().get(DcMotorEx.class,"lsFront"),
        opModeUtilities.getHardwareMap().get(DcMotorEx.class, "lsBack"));

        lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lsFront.setTargetPosition(0);
        lsBack.setTargetPosition(0);

        lsFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lsBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lsFront.setDirection(DcMotor.Direction.REVERSE);
        lsBack.setDirection(DcMotor.Direction.REVERSE);

        lsFront.setPositionPIDFCoefficients(30);
        lsFront.setVelocityPIDFCoefficients(5, 0, 0, 12);

        lsBack.setPositionPIDFCoefficients(30);
        lsBack.setVelocityPIDFCoefficients(5, 0, 0, 12);

        lsFront.setPower(1);
        lsBack.setPower(1);
    }
    @Override
    protected boolean isBusy() {
        //return Math.abs(getTargetPositionMeters() - getPositionMeters()) < 0.001 && Math.abs(getVelocityMeters()) <
        // 0.001;
        return lsFront.isBusy() || lsBack.isBusy();
    }
    @Override
    protected void setTargetPositionMeters(double position) {
        lsFront.setTargetPosition((int) (position * TICKS_PER_METER));
        lsBack.setTargetPosition((int) (position * TICKS_PER_METER));
    }

    @Override
    public double getTargetPositionMeters() {
        return lsFront.getTargetPosition() / TICKS_PER_METER;
    }

    @Override
    public double getPositionMeters() {
        return lsFront.getCurrentPosition() / TICKS_PER_METER;
    }

    @Override
    public double getVelocityMeters() {
        return this.lsFront.getVelocity() / TICKS_PER_METER;
    }
}
