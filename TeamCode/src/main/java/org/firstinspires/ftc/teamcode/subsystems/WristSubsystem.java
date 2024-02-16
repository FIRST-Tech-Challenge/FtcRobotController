package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.Locale;

public class WristSubsystem extends SubsystemBase {

    private final DcMotorEx wrist;
    private final boolean continuousMode;
    private boolean zeroPositionMode = false;

    public static final float POWER_SCALE = 50;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("WristSubsystem");

    public WristSubsystem(final DcMotorEx wrist, boolean continuousMode) {
        this.wrist = wrist;
        this.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.continuousMode = continuousMode;
    }

    public void moveWrist(double frontward, double backward) {
        double power = frontward - backward;
        dbp.debug(String.format(Locale.ENGLISH, "Power: %f", power), true);
        dbp.debug("Servo position: " + getPosition(), true);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setPower(power/POWER_SCALE);
    }

    public void setWristPosition(int position) {
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPower(1f/POWER_SCALE);
        wrist.setTargetPosition(position);
    }

    public double getPosition() {
        return wrist.getCurrentPosition();
    }

}
