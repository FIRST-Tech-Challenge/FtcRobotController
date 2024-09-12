package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor bucketArmMotor; // Bucket arm
    private final DcMotor horizontalArmMotor; //
    private final DcMotor hangingArmMotor;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("ArmSubsystem");

    public ArmSubsystem(final HashMap<RobotHardwareInitializer.Arm, DcMotor> ARM) {
        this.bucketArmMotor = ARM.get(RobotHardwareInitializer.Arm.ARM1);
        assert this.bucketArmMotor != null;
        bucketArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bucketArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.horizontalArmMotor = ARM.get(RobotHardwareInitializer.Arm.ARM2);
        assert this.horizontalArmMotor != null;
        horizontalArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.hangingArmMotor = ARM.get(RobotHardwareInitializer.Arm.ARM3);
        assert this.hangingArmMotor != null;
        hangingArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hangingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void manualMoveBucketArm(double power) {
        bucketArmMotor.setPower(power);
    }

    public void manualMoveHorizontalArm(double power) {
        horizontalArmMotor.setPower(power);
    }

    public void manualMoveHangingArm(double power) {
        hangingArmMotor.setPower(power);
    }

    /**
     * Sets the power of the armMotor to zero
     */
    public void haltArm() {
        bucketArmMotor.setPower(0);
        horizontalArmMotor.setPower(0);
        hangingArmMotor.setPower(0);
    }

    public void resetArm() {
        bucketArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
