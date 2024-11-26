package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.MIN_POWER;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotor lF, rF, lB, rB, eL, eB, eR;
    private final double INCHES_PER_TICK = 0.0018912d;
    public final ElapsedTime elapsedTime;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriveSubsystem");

    public DriveSubsystem(HashMap<RobotHardwareInitializer.Component, DcMotor> driveMotors) {
        this(driveMotors.get(RobotHardwareInitializer.MotorComponent.LEFT_FRONT),
                driveMotors.get(RobotHardwareInitializer.MotorComponent.RIGHT_FRONT),
                driveMotors.get(RobotHardwareInitializer.MotorComponent.LEFT_BACK),
                driveMotors.get(RobotHardwareInitializer.MotorComponent.RIGHT_BACK),
                driveMotors.get(RobotHardwareInitializer.EncoderComponent.ENCODER_LEFT),
                driveMotors.get(RobotHardwareInitializer.EncoderComponent.ENCODER_BACK),
                driveMotors.get(RobotHardwareInitializer.EncoderComponent.ENCODER_RIGHT));
    }

    public DriveSubsystem(final DcMotor leftFront, final DcMotor rightFront,
                          final DcMotor leftBack, final DcMotor rightBack,
                          final DcMotor encoderLeft, final DcMotor encoderBack,
                          final DcMotor encoderRight) {
        dbp.createNewTelePacket();
        dbp.info(leftFront+", "+leftBack+", "+rightFront+", "+rightBack);
        dbp.send(false);

        lF = leftFront;
        rF = rightFront;
        lB = leftBack;
        rB = rightBack;

        eL = encoderLeft;
        eB = encoderBack;
        eR = encoderRight;

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @SuppressLint("DefaultLocale")
    public boolean moveRobot(double axial, double lateral, double yaw) {

        dbp.createNewTelePacket();
        dbp.info("Moving robot in subsystem: "+axial+", "+lateral+", "+yaw);
        dbp.send(true);

        double max;

        double leftFrontPower  = axial - lateral - yaw;
        double rightFrontPower = axial + lateral + yaw;
        double leftBackPower   = axial + lateral - yaw;
        double rightBackPower  = axial - lateral + yaw;

        dbp.createNewTelePacket();

        dbp.debug("LF: " + String.format("%f\n", leftFrontPower), false);
        dbp.debug("RF: " + String.format("%f\n", rightFrontPower), false);
        dbp.debug("LB: " + String.format("%f\n", leftBackPower), false);
        dbp.debug("RB: " + String.format("%f\n", rightBackPower), true);

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        lF.setPower(leftFrontPower);
        rF.setPower(rightFrontPower);
        lB.setPower(leftBackPower);
        rB.setPower(rightBackPower);
        return true;
    }

    /**
     * Moves the robot by a specified distance
     * @param axial Forward/Backward Movement
     * @param lateral Left/Right Movement
     * @param yaw Rotation
     * @param distance Distance to move (in inches)
     */
    public void moveRobotByDistance(double axial, double lateral, double yaw, double distance) {
        moveRobot(axial, lateral, yaw);

        if ((axial + lateral + yaw) == 0) return;
        if (distance == 0) return;

        // TODO: implement directionality
        final int DIRECTION = (distance < 0) ? -1 : 1;

        // Axial = y, Lateral = x, yaw = z
        double[] startingPosition = new double[3];
        startingPosition[0] = eL.getCurrentPosition() * INCHES_PER_TICK;
        startingPosition[1] = eB.getCurrentPosition() * INCHES_PER_TICK;
        startingPosition[2] = eR.getCurrentPosition() * INCHES_PER_TICK;

        while (lF.isBusy()) {
            double[] newPositions = new double[3];
            newPositions[0] =
                    ((eL.getCurrentPosition() * INCHES_PER_TICK) * DIRECTION) - startingPosition[0];
            newPositions[1] =
                    ((eB.getCurrentPosition() * INCHES_PER_TICK) * DIRECTION) - startingPosition[1];
            newPositions[2] =
                    ((eR.getCurrentPosition() * INCHES_PER_TICK) * DIRECTION) - startingPosition[2];

            if ((newPositions[0] + newPositions[1] + newPositions[2]) >= distance) {
                resetDriveMotors();
                return;
            }
        }
    }

    public void resetDriveMotors() {
        dbp.debug("Resetting Drive Motors...");
        lF.setPower(MIN_POWER);
        rF.setPower(MIN_POWER);
        lB.setPower(MIN_POWER);
        rB.setPower(MIN_POWER);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
