package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Lift {
    //TODO change values to real ones (touch grass)
    private final double AMP_LIMIT_LIFT = 0;
    private final double AMP_LIMIT_ASCEND = 0;
    private final double LIFT_POWER = 1;
    private final double ASCEND_POWER = -LIFT_POWER;
    DcMotorEx liftMotor = null;
    Telemetry telemetry;
    private boolean finishedHangingNot = true;
    private double power = 0;
    private int ascendLevel = 1;
    private boolean IS_DEBUG = false;

    public Lift(OpMode opMode, boolean IS_DEBUG) {
        setTelemetry(telemetry);
        setIS_DEBUG(IS_DEBUG);
        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        if (IS_DEBUG) {
            opMode.telemetry.addData("LiftConstructor", true);
        }
        init();
    }

    public void init() {
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        resetEncoders();
        if (IS_DEBUG) {
            telemetry.addData("LiftInit", true);
        }
    }

    public Action hanging() {
        if (IS_DEBUG) {
            telemetry.addData("LiftEncoder", liftMotor.getCurrentPosition());
        }
        return new setPowerAction();

    }

    public void resetEncoders() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }

    public void setLiftMotor(DcMotorEx liftMotor) {
        this.liftMotor = liftMotor;
    }

    public boolean isFinishedHangingNot() {
        return finishedHangingNot;
    }

    public void setFinishedHangingNot(boolean finishedHangingNot) {
        this.finishedHangingNot = finishedHangingNot;
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public double getLIFT_POWER() {
        return LIFT_POWER;
    }

    public double getASCEND_POWER() {
        return ASCEND_POWER;
    }

    public int getAscendLevel() {
        return ascendLevel;
    }

    public void setAscendLevel(int ascendLevel) {
        this.ascendLevel = ascendLevel;
    }

    public boolean isIS_DEBUG() {
        return IS_DEBUG;
    }

    public void setIS_DEBUG(boolean IS_DEBUG) {
        this.IS_DEBUG = IS_DEBUG;
    }

    public double getAMP_LIMIT_LIFT() {
        return AMP_LIMIT_LIFT;
    }

    public double getAMP_LIMIT_ASCEND() {
        return AMP_LIMIT_ASCEND;
    }

    public class setPowerAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (liftMotor.getCurrent(CurrentUnit.AMPS) < AMP_LIMIT_LIFT && (ascendLevel == 1 || ascendLevel == 3)) {
                liftMotor.setPower(LIFT_POWER);
            } else if (liftMotor.getCurrent(CurrentUnit.AMPS) >= AMP_LIMIT_LIFT && ascendLevel == 1) {
                ascendLevel = 2;
            } else if (liftMotor.getCurrent(CurrentUnit.AMPS) < AMP_LIMIT_ASCEND && (ascendLevel == 2 || ascendLevel == 4)) {
                liftMotor.setPower(ASCEND_POWER);
            } else if (liftMotor.getCurrent(CurrentUnit.AMPS) >= AMP_LIMIT_ASCEND && ascendLevel == 2) {
                ascendLevel = 3;
            } else if (liftMotor.getCurrent(CurrentUnit.AMPS) >= AMP_LIMIT_LIFT && ascendLevel == 3) {
                ascendLevel = 4;
            } else if (liftMotor.getCurrent(CurrentUnit.AMPS) >= AMP_LIMIT_ASCEND && ascendLevel == 4) {
                finishedHangingNot = false;
            }
            if (IS_DEBUG) {
                telemetry.addData("Ascend Level", ascendLevel);
                telemetry.addData("Power", liftMotor.getPower());
                telemetryPacket.put("Ascend Level", ascendLevel);
                telemetryPacket.put("Power", liftMotor.getPower());
                telemetry.addData("FinishedHangingNot", finishedHangingNot);
                telemetryPacket.put("FinishedHangingNot", finishedHangingNot);
            }
            return finishedHangingNot;

        }
    }
}