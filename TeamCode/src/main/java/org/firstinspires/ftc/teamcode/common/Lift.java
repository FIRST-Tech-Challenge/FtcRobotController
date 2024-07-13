package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware_data.GoBilda435DcMotorData;

@Config
public class Lift extends Component {
    private final DcMotorEx liftMotorL;
    private final DcMotorEx liftMotorR;
    private final PIDFController pidfL;
    private final PIDFController pidfR;
    public static double kP = 0.02;
    //original kP = 0.025
    public static double kI = 0.0;
    public static double kD = 0.0002;
    //original kD = 0.0005
    public static double kF = 0.0001;
    private final double positionTolerance = 10;
    private final double derivativeTolerance = 10;
    private final double maxVelocity = GoBilda435DcMotorData.maxTicksPerSec;
    private final int maxPos = 2000;
    private final int retractPos = 800;
    private final int cruisePos = 950;
    private final int deploy1Pos = 725;
    private final int groundPlacementPos = 150;
    private final int loadPos = 105;
    private final int minPos = 0;
    public static int targetPos;
    private int autoOffset = 0;
    private int manualOffset = 25;
    private final double defaultMaxPower = 0.90;
    private double maxPower = defaultMaxPower;
    public static int currentPos;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn) {
        super(telemetry, loggingOn);
        pidfL = new PIDFController(kP, kI, kD, kF);
        pidfL.setTolerance(positionTolerance);
        pidfR = new PIDFController(kP, kI, kD, kF);
        pidfR.setTolerance(positionTolerance);

        liftMotorL = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftMotorR = hardwareMap.get(DcMotorEx.class, "liftRight");

        liftMotorL.setDirection(DcMotor.Direction.FORWARD);
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPos = 0;
    }

    public void update() {
//        pidfL.setPIDF(kP,kI,kD,kF);
//        pidfR.setPIDF(kP,kI,kD,kF);

        setPIDFMotorPower();
        if (loggingOn) {
            logTelemetry();
        }
    }

    public void manualUp(double power) {
        if (!atTop(manualOffset)) {
            setMotorsPower(power);
        } else {
            stop();
        }
    }

    public void manualDown(double power) {
        if (!atBottom(manualOffset)) {
            setMotorsPower(-power);
        } else {
            stop();
        }
    }

    public void stop() {
        setTargetPos(liftMotorL.getCurrentPosition());
    }

    public void goToRetractPosition() {
        setTargetPos(retractPos);
    }

    public void goToLoadPosition() {
        setTargetPos(loadPos);
    }

    public void goToMinPosition() {
        setTargetPos(minPos);
    }

    public void goToDeploy1Position() {
        setTargetPos(deploy1Pos);
    }

    public void goToCruisePosition() {
        setTargetPos(cruisePos);
    }

    public void goToGroundPlacementPosition() {
        setTargetPos(groundPlacementPos);
    }

    private boolean atTop(int offset) {
        if ((liftMotorL.getCurrentPosition() - offset) >= maxPos) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isBusy() {
        return (!pidfL.atSetPoint() || !pidfR.atSetPoint());
    }

    private boolean atBottom(int offset) {
        if ((liftMotorL.getCurrentPosition() + offset) <= minPos) {
            return true;
        } else {
            return false;
        }
    }

    public void setMotorsPower(double power) {
        liftMotorL.setPower(power * maxPower);
        liftMotorR.setPower(power * maxPower);
    }

    public void setLMotorPower(double power) {
        liftMotorL.setPower(power * maxPower);
    }

    public void setRMotorPower(double power) {
        liftMotorR.setPower(power * maxPower);
    }

    public void setTargetPos(int targetPos) {
        if (targetPos < minPos) {
            this.targetPos = minPos;
        } else if (targetPos > maxPos) {
            this.targetPos = maxPos;
        } else {
            this.targetPos = targetPos;
        }
    }

    private void setPIDFMotorPower() {
        setLMotorPower(pidfL.calculate(liftMotorL.getCurrentPosition(), targetPos));
        setRMotorPower(pidfR.calculate(liftMotorR.getCurrentPosition(), targetPos));
    }

    private void logTelemetry() {
        telemetry.addData("PositionL:  ", liftMotorL.getCurrentPosition());
        telemetry.addData("PositionR:  ", liftMotorR.getCurrentPosition());
        telemetry.addData("Target:  ", targetPos);
        telemetry.addData("PowerL:  ", liftMotorL.getPower());
        telemetry.addData("PowerR:  ", liftMotorR.getPower());
        telemetry.addData("Busy:  ", isBusy());
        //        telemetry.update();
    }
}