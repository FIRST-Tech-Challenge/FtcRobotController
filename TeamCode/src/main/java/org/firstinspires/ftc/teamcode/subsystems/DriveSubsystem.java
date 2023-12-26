package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotOpMode.MIN_POWER;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotor lF, rF, lB, rB;
    public final ElapsedTime elapsedTime;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriveSubsystem");

    public DriveSubsystem(HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors) {
        this(driveMotors.get(RobotHardwareInitializer.DriveMotor.LEFT_FRONT),
                driveMotors.get(RobotHardwareInitializer.DriveMotor.RIGHT_FRONT),
                driveMotors.get(RobotHardwareInitializer.DriveMotor.LEFT_BACK),
                driveMotors.get(RobotHardwareInitializer.DriveMotor.RIGHT_FRONT));
    }

    public DriveSubsystem(final DcMotor leftFront, final DcMotor rightFront,
                          final DcMotor leftBack, final DcMotor rightBack) {
        dbp.createNewTelePacket();
        dbp.info(leftFront+", "+leftBack+", "+rightFront+", "+rightBack);
        dbp.send(true);
        lF = leftFront;
        rF = rightFront;
        lB = leftBack;
        rB = rightBack;
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
