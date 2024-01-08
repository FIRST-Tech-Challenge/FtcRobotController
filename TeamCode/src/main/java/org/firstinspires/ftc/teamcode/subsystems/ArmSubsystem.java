package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.Locale;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor armMotor;
    private final int POSITION_MOVE_POWER = 1;
    private final int ARM_ANGLE_OFFSET = 0;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("ArmSubsystem");

    public ArmSubsystem(DcMotor armMotor) {
        this.armMotor = armMotor;
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public enum Direction {
        FRONTWARD,
        BACKWARD,
    }

    public enum ArmPositions {
        ZERO(0),
        BOARD(135);

        private final int position;

        ArmPositions(final int position) {
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
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
    }

    public void manualMoveArm(double power) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
    }

    /**
     * @param position The position that the arm will move towards.
     */
    public void positionMoveArm(final int position) {
        armMotor.setTargetPosition(position - ARM_ANGLE_OFFSET);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(2); //POSITION_MOVE_POWER);
        while (armMotor.isBusy()) {
            dbp.debug("Waiting for motor to move to position", true);
            dbp.debug(String.format(Locale.ENGLISH,
                    "Arm position: %d", armMotor.getCurrentPosition()));
        }
    }

    /**
     * @param position The position that the arm will move towards.
     */
    public void positionMoveArm(final ArmPositions position) {
        positionMoveArm(position.getPosition());
    }

    /**
     * Sets the power of the armMotor to zero
     */
    public void haltArm() {
        armMotor.setPower(0);
    }

    public void resetArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
