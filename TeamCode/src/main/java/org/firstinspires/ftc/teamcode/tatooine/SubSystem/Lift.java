package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
    private double power = 0;
    private int ascendLevel = 1;
    private boolean IS_DEBUG = false;

    //lift constructor
    public Lift(OpMode opMode, boolean IS_DEBUG) {
        setTelemetry(telemetry);
        setIS_DEBUG(IS_DEBUG);
        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        if (IS_DEBUG) {
            opMode.telemetry.addData("LiftConstructor", true);
        }
        init();
    }

    //init that resets encoders and sets direction
    public void init() {
        //TODO change directions if needed
        //liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        resetEncoders();
        if (IS_DEBUG) {
            telemetry.addData("LiftInit", true);
        }
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

    //an actions that does the hanging
    public Action hanging() {
        return new Ascend();
    }

    public class Ascend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //as long as the level of the hanging is 1 or 3 and the ampere level is lower then the max level of the lift ampere the lift goes up
            if (liftMotor.getCurrent(CurrentUnit.AMPS) < AMP_LIMIT_LIFT && (ascendLevel == 1 || ascendLevel == 3)) {
                liftMotor.setPower(LIFT_POWER);
                //if lift reached max height ascend to next level
            } else if (liftMotor.getCurrent(CurrentUnit.AMPS) >= AMP_LIMIT_LIFT && (ascendLevel == 1 || ascendLevel == 3)) {
                ascendLevel = ascendLevel + 1;
                //as long as the lift didn't finish to ascend(go down) and it supposed to the it's ascending
            } else if (liftMotor.getCurrent(CurrentUnit.AMPS) < AMP_LIMIT_ASCEND && (ascendLevel == 2 || ascendLevel == 4)) {
                liftMotor.setPower(ASCEND_POWER);
                //if lift finished to ascend (go down) then ascend to next level
            } else if (liftMotor.getCurrent(CurrentUnit.AMPS) >= AMP_LIMIT_ASCEND && (ascendLevel == 2 || ascendLevel == 4)) {
                ascendLevel = ascendLevel + 1;
                //if the lift finished the hanging then stop hanging
            }
            if (IS_DEBUG) {
                telemetry.addData("Ascend Level", ascendLevel);
                telemetry.addData("Power", liftMotor.getPower());
                telemetryPacket.put("Ascend Level", ascendLevel);
                telemetryPacket.put("Power", liftMotor.getPower());
            }
            return ascendLevel != 5;

        }
    }
}