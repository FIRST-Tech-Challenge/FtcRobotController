package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.Locale;

public class WristSubsystem extends SubsystemBase {

    private final DcMotorEx wrist;
    private final boolean continuousMode;
    private boolean zeroPositionMode = false;

    public static final float POWER_SCALE = 1;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("WristSubsystem");

    public WristSubsystem(final DcMotorEx wrist, boolean continuousMode) {
        this.wrist = wrist;
        this.wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.wrist.setDirection(DcMotorSimple.Direction.REVERSE);
        this.continuousMode = continuousMode;
    }

    public void moveWrist(double frontward, double backward) {
        double power = frontward - backward;
        dbp.debug(String.format(Locale.ENGLISH, "Power: %f", power), true);
        dbp.debug("Servo position: " + getPosition(), true);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setVelocity(1*power, AngleUnit.DEGREES);
    }

    public void setWristPosition(int position) {
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPower(1f/POWER_SCALE);
        wrist.setTargetPosition(-position);
    }

    public double getPosition() {
        return wrist.getCurrentPosition();
    }
    public boolean isBusy() { return wrist.isBusy(); }
    public DcMotorEx getWrist() { return wrist; }

}
