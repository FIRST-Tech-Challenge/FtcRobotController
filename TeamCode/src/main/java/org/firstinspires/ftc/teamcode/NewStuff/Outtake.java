package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Outtake {

    public static final double P_CONSTANT = 0.01;
    private final OpModeUtilities opModeUtilities;
    public DcMotor lsFront, lsBack;
    public Servo tray;
    public Servo clamp;
    public Servo hook;
    public Servo pigeonHeadServo;
    public OuttakeState state;

    LinearOpMode opMode;
    private double CLAMP_OPEN_POS = 0.471;
    private double CLAMP_CLOSE_POS = 0.57;
    private double TRAY_INTAKE_POS = 0.3;
    private double TRAY_OUTTAKE_POS = 0;

    private double PIGEON_HEAD_DEFAULT_POS = 0.5;

    private double PIGEON_HEAD_ANGLE_MAX = PIGEON_HEAD_DEFAULT_POS + 0.18;
    private double PIGEON_HEAD_ANGLE_MIN = PIGEON_HEAD_DEFAULT_POS - 0.18;

    public Outtake(OpModeUtilities opModeUtilities) {

        this.opModeUtilities = opModeUtilities;
        this.state = new OuttakeState();
        setUpHardware();

    }

    private void setUpHardware() {

        lsFront = opModeUtilities.getHardwareMap().dcMotor.get("lsFront");
        lsBack = opModeUtilities.getHardwareMap().dcMotor.get("lsBack");
        tray = opModeUtilities.getHardwareMap().servo.get("arm");
        clamp = opModeUtilities.getHardwareMap().servo.get("holderClamp");
        hook = opModeUtilities.getHardwareMap().servo.get("linearLocker");
        pigeonHeadServo = opModeUtilities.getHardwareMap().servo.get("trayAngle");

        lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lsBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double lsToTicksCalcPowerParallelSequence(GenericState conditionState, double maxPower) {
        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {
                return Range.clip(state.getError() * P_CONSTANT, -maxPower, maxPower);
            }
        }
        return 0;
    }

    public void lsMove (double power) {
        lsFront.setPower(power);
        lsBack.setPower(power);
    }

    public void setState (double currentTicks, double currentTargetTicks) {
        state.setCurrentTargetTicks(currentTargetTicks);
        state.setCurrentTicks(currentTicks);
    }

    public void update(GenericState conditionState) {
        if (!this.state.isDone()) {
            lsMove(lsToTicksCalcPowerParallelSequence(conditionState, 1));
        }
    }

    public void trayToIntakePos(boolean blocking) {
        //backup value 0.45
        //delta 0.3
        tray.setPosition(getTRAY_INTAKE_POS());
        if (blocking) {
            opMode.sleep(500);
        }
    }

    public void trayToOuttakePos(boolean blocking) {
        //backup value 0.15
        //delta 0.3
        tray.setPosition(getTRAY_OUTTAKE_POS());
        if (blocking) {
            opMode.sleep(100);
        }
    }

    public void closeClamp(boolean blocking) {
        clamp.setPosition(getCLAMP_CLOSE_POS());
        if (blocking) {
            opMode.sleep(300);
        }
    }

    public void openClamp(boolean wide, boolean auto, boolean blocking) {
        if (wide) {
            if (auto) {
                clamp.setPosition(getCLAMP_OPEN_POS());
            }
            else {
                clamp.setPosition(getCLAMP_OPEN_POS());
            }
        } else {
            clamp.setPosition(0.51);
        }

        if (blocking) {
            opMode.sleep(300);
        }
    }

    public double getCLAMP_OPEN_POS () {
        return CLAMP_OPEN_POS;
    }

    public double getCLAMP_CLOSE_POS() {
        return CLAMP_CLOSE_POS;
    }

    public double getTRAY_INTAKE_POS() {
        return TRAY_INTAKE_POS;
    }

    public double getTRAY_OUTTAKE_POS() {
        return TRAY_OUTTAKE_POS;
    }
}
