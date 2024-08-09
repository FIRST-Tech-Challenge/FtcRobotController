package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PIDController;

public class DriveTrain {
    private final OpModeUtilities opModeUtilities;
    private final DcMotor testMotorDeleteLater;
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final DcMotor backEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;

    public DrivetrainState state;

    private PIDController straightController = new PIDController("straight", 0.005, 0.0000015, 0.8, false);;
    private PIDController fLeftMecanumController = new PIDController("fl mecanum", 0.005, 0.0000005, 0.4, true); //0.01 - 0.0001;
    private PIDController bRightMecanumController = new PIDController("br mecanum", 0.005, 0.0000005, 0.4, true);;
    PIDController setHeadingController = new PIDController("set heading", 0.06, 0, 2_500_000, false);

    public DriveTrain(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.state = new DrivetrainState();
        fLeft = opModeUtilities.getHardwareMap().dcMotor.get("fLeft");
        fRight = opModeUtilities.getHardwareMap().dcMotor.get("fRight");
        bLeft = opModeUtilities.getHardwareMap().dcMotor.get("bLeft");
        bRight = opModeUtilities.getHardwareMap().dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightEncoder = bRight;
        leftEncoder = bLeft;
        backEncoder = fRight;

        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        testMotorDeleteLater = opModeUtilities.getHardwareMap().dcMotor.get("testMotor");
    }
    public void setMotorPower(final DrivetrainPowers drivetrainPowers) {
        setMotorPower(drivetrainPowers.fLeftPower, drivetrainPowers.fRightPower, drivetrainPowers.bLeftPower, drivetrainPowers.bRightPower);
    }

    public void setMotorPower(double fLeft, double fRight, double bLeft, double bRight) {
        this.fLeft.setPower(fLeft);
        this.bRight.setPower(bRight);
        this.bLeft.setPower(bLeft);
        this.fRight.setPower(fRight);
    }

    public void setMotorPower(DcMotor dcMotor, double power) {
        dcMotor.setPower(power);
    }

    public DrivetrainPowers straightParallelPowerPISequence(GenericState conditionState, double maxPower) {

        double power;

        if (conditionState.isDone()) {
            if (!state.isDone() && state.getOpMode().opModeIsActive()) {

                power = straightController.calculatePID(state.getCurrentTargetTicks(0), state.getCurrentTargetTicks(0));

                //cap power
                power = Range.clip(power, -1 * maxPower, maxPower);

                return new DrivetrainPowers(power, power, power, power);

            }
        }

        return new DrivetrainPowers(0, 0, 0, 0);
    }

    public void setFLeftPower(double power) { fLeft.setPower(power); }
    public void setFRightPower(double power) { fRight.setPower(power); }
    public void setBLeftPower(double power) { bLeft.setPower(power); }
    public void setBRightPower(double power) { bRight.setPower(power); }
    public void setPower (double fLeftPower, double fRightPower, double bLeftPower, double bRightPower){
        setFLeftPower(fLeftPower);
        setFRightPower(fRightPower);
        setBLeftPower(bLeftPower);
        setBRightPower(bRightPower);
    }
    public DcMotor getBackEncoder() {
        return backEncoder;
    }
    public DcMotor getRightEncoder() {
        return rightEncoder;
    }
    public DcMotor getLeftEncoder() {
        return leftEncoder;
    }
    public DcMotor getfLeft() {
        return fLeft;
    }
    public DcMotor getfRight() {
        return fRight;
    }
    public DcMotor getbLeft() {
        return bLeft;
    }
    public DcMotor getbRight() {
        return bRight;
    }
    public DcMotor getTestMotorDeleteLater() {
        return testMotorDeleteLater;
    }
    public void setTestPower(double power) throws InterruptedException {
        getTestMotorDeleteLater().setPower(power);
        Thread.sleep(3000);
        getTestMotorDeleteLater().setPower(0);
    }
}
