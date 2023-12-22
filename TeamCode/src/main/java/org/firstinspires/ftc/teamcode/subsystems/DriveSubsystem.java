package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotOpMode.MIN_POWER;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotor lF, rF, lB, rB;
    public final ElapsedTime elapsedTime;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriveSubsystem");

    public DriveSubsystem(final DcMotor leftFront, final DcMotor rightFront,
                          final DcMotor leftBack, final DcMotor rightBack, final HardwareMap hMap) {
        dbp.createNewTelePacket();
        lF = leftFront;
        rF = rightFront;
        lB = leftBack;
        rB = rightBack;
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @SuppressLint("DefaultLocale")
    public boolean moveRobot(double axial, double lateral, double yaw, long endTime) {

        double max;
        if(elapsedTime.now(TimeUnit.NANOSECONDS) >= endTime) {
            resetDriveMotors();
            return false;
        }

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
}
