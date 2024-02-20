package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
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

    public static final float ANGLE_PER_SECOND = 60;

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
        setVelocity(power);
    }

    public void setWristPosition(int position) {
        int adjustedPosition = positionFromAngle(position);

        wrist.setTargetPosition(adjustedPosition);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setVelocity(1f);
        System.out.println("POSITION DATA:"+adjustedPosition+", "+position+", "+wrist.getMotorType().getTicksPerRev()+", "+wrist.getMotorType().getGearing());
    }

    private void setVelocity(double scale) {
        wrist.setVelocity(ANGLE_PER_SECOND*scale, AngleUnit.DEGREES);
    }

    public int positionFromAngle(double angle) {
        double ticksPerRevolution = wrist.getMotorType().getTicksPerRev();
        double ticksPerAngle = ticksPerRevolution/360d;
        double adjustedPosition = ticksPerAngle*angle;
        return (int) adjustedPosition;
    }

    public double getPosition() {
        return wrist.getCurrentPosition();
    }
    public boolean isBusy() { return wrist.isBusy(); }
    public DcMotorEx getWrist() { return wrist; }

}
