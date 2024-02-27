package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
import java.util.Locale;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor armMotor1;
    private final DcMotor armMotor2;

    private final Double SECOND_MOTOR_POWER = 1/5.25;

    private final int POSITION_MOVE_POWER = 1;
    private final int ARM_ANGLE_OFFSET = 0;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("ArmSubsystem");

    public ArmSubsystem(final HashMap<RobotHardwareInitializer.Arm, DcMotor> ARM) {
        this.armMotor1 = ARM.get(RobotHardwareInitializer.Arm.ARM1);
        assert this.armMotor1 != null;
        armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.armMotor2 = ARM.get(RobotHardwareInitializer.Arm.ARM2);
        assert this.armMotor2 != null;
        armMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public enum Direction {
        FRONTWARD,
        BACKWARD,
    }

    public enum ArmPosition {
        FLOOR(0),
        BOARD((40+90+45));

        private final int position;

        ArmPosition(final int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }
    }

    /**
     * @param direction The direction that the arm will move towards.
     * @param power A power value set within the interval [0, 1]
     */
    public void manualMoveArm(Direction direction, double power) {
        power = Math.max(0, Math.min(1, power));
        power *= (direction == Direction.FRONTWARD) ? 1f : -1f;
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor1.setPower(power);

        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setPower(power * SECOND_MOTOR_POWER);
    }

    public void manualMoveArm(double power) {
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor1.setPower(power);

        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setPower(power * SECOND_MOTOR_POWER);
    }

    /**
     * @param position The position that the arm will move towards. Takes in DEGREES
     */
    public void positionMoveArm(final double position) {
        int calculatedPosition = positionFromAngle(position - ARM_ANGLE_OFFSET, AngleUnit.DEGREES);
        // calculated the error in the arm and multiplied by it
        // (may be an issue of gear ratios in the end)
        calculatedPosition = (int) (4.375d*calculatedPosition);
        armMotor1.setTargetPosition(calculatedPosition);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1.setPower(1); //POSITION_MOVE_POWER);

        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setPower(SECOND_MOTOR_POWER);

        if (armMotor1.isBusy()) {
            dbp.debug("Waiting for motor to move to position", true);
            dbp.debug(String.format(Locale.ENGLISH,
                    "Arm position: %d", armMotor1.getCurrentPosition()));
        }
    }

    /**
     * Converts angle units into position units usable by {@link DcMotor#setTargetPosition(int)}
     * @param angle The angle to convert.
     * @param angleUnit The unit of the angle.
     * @return The angle converted into position units.
     */
    public int positionFromAngle(double angle, AngleUnit angleUnit) {
        double ticksPerRevolution = armMotor1.getMotorType().getTicksPerRev();
        double scale = angleUnit.toDegrees(angle)/360;
        return (int) (ticksPerRevolution*scale);
    }

    public static int positionFromAngle(DcMotor motor, double angle, AngleUnit angleUnit) {
        double ticksPerRevolution = motor.getMotorType().getTicksPerRev();
        double scale = angleUnit.toDegrees(angle)/360;
        return (int) (ticksPerRevolution*scale);
    }

    /**
     * @param position The position that the arm will move towards.
     */
    public void positionMoveArm(final ArmPosition position) {
        positionMoveArm(position.getPosition());
    }

    /**
     * Sets the power of the armMotor to zero
     */
    public void haltArm() {
        armMotor1.setPower(0);
        armMotor2.setPower(0);
    }

    public void resetArm() {
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotor getArmMotor1() {
        return armMotor1;
    }

    public DcMotor getArmMotor2() {
        return armMotor2;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
