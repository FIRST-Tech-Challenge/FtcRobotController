package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.MatchRecorder.MatchLogger;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.MotorTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.ServoTypeValue;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
import java.util.Objects;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor bucketArmMotor; // Bucket arm
    private final DcMotor horizontalArmMotor; //
    private final DcMotor hangingArmMotor;
    private final Servo bucketArmServo;

    public static final float ANGLE_PER_SECOND = 60;

    // TODO: Clarify what this enum does, (renamed it from ARMS to ArmState because it seems to fit better)
    public enum ArmState {
        BUCKET_ARM,
        HORIZONTAL_ARM,
        HANGING_ARM,
        ALL;
    }

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("ArmSubsystem");

    public ArmSubsystem(final HashMap<RobotHardwareInitializer.Arm, DynamicTypeValue> ARM) {
        this.bucketArmMotor = ((MotorTypeValue) Objects.requireNonNull(ARM.get(RobotHardwareInitializer.Arm.BUCKET_ARM_MOTOR))).getValue();
        assert this.bucketArmMotor != null;
        bucketArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bucketArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.horizontalArmMotor = ((MotorTypeValue) Objects.requireNonNull(ARM.get(RobotHardwareInitializer.Arm.ARM2))).getValue();
        // assert this.horizontalArmMotor != null;
        horizontalArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hangingArmMotor = ((MotorTypeValue) Objects.requireNonNull(ARM.get(RobotHardwareInitializer.Arm.ARM3))).getValue();
        // assert this.hangingArmMotor != null;
        hangingArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hangingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.bucketArmServo = ((ServoTypeValue) Objects.requireNonNull(ARM.get(RobotHardwareInitializer.Arm.BUCKET_SERVO))).getValue();
        assert this.bucketArmServo != null;
    }

    /**
     * Sets all arms power to the inputted value
     * @param power the power to set the arms to
     */
    public void manualMoveArm(double power) {
        // when would we need to move all of the arms?
        manualMoveArm(power, ArmState.ALL);
    }

    /**
     * Sets a specified arm's power
     * @param power the power to set the arm to
     * @param arm the arm to set the power to
     */
    public void manualMoveArm(double power, ArmState arm) {
        MatchLogger.getInstance().logArm(this, power, arm);
        switchArmMode(DcMotor.RunMode.RUN_USING_ENCODER, arm);
        switch (arm) {
            case ALL:
                manualMoveHorizontalArm(power);
                manualMoveHangingArm(power);
                manualMoveBucketArm(power);
                break;
            case BUCKET_ARM:
                manualMoveBucketArm(power);
                break;
            case HANGING_ARM:
                manualMoveHangingArm(power);
                break;
            case HORIZONTAL_ARM:
                manualMoveHorizontalArm(power);
                break;
            default:
                dbp.error("Unknown arm type: " + arm);
                break;
        }
    }

    public void manualMoveBucketArm(double power) {
        MatchLogger.getInstance().logArm(this, power);
        bucketArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketArmMotor.setPower(power);
    }

    public void manualMoveHorizontalArm(double power) {
        MatchLogger.getInstance().logArm(this, power);
        horizontalArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalArmMotor.setPower(power);
    }

    public void manualMoveHangingArm(double power) {
        MatchLogger.getInstance().logArm(this, power);
        hangingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangingArmMotor.setPower(power);
    }

    public void switchArmMode(DcMotor.RunMode runMode) {
        switchArmMode(runMode, ArmState.ALL);
    }

    public void switchArmMode(DcMotor.RunMode runMode, ArmState arm) {
        MatchLogger.getInstance().logArm(this, arm);
        switch (arm) {
            case ALL:
                horizontalArmMotor.setMode(runMode);
                bucketArmMotor.setMode(runMode);
                hangingArmMotor.setMode(runMode);
                break;
            case BUCKET_ARM:
                bucketArmMotor.setMode(runMode);
                break;
            case HANGING_ARM:
                hangingArmMotor.setMode(runMode);
                break;
            case HORIZONTAL_ARM:
                horizontalArmMotor.setMode(runMode);
                break;
            default:
                dbp.error("Unknown arm type: " + arm);
                break;
        }
    }

    /**
     * Sets the power of the armMotor to zero
     */
    public void haltAllArms() {
        MatchLogger.getInstance().logArm(this);
        switchArmMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketArmMotor.setPower(0);
        horizontalArmMotor.setPower(0);
        hangingArmMotor.setPower(0);
    }

    public void resetAllArms() {
        MatchLogger.getInstance().logArm(this);
        switchArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotor getBucketArmMotor() {
        return bucketArmMotor;
    }

    public DcMotor getHorizontalArmMotor() {
        return horizontalArmMotor;
    }

    public DcMotor getHangingArmMotor() {
        return hangingArmMotor;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
