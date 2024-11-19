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
import org.firstinspires.ftc.teamcode.util.MatchRecorder.MatchLogger;
import org.opencv.core.Mat;

import java.util.Locale;

public class WristSubsystem extends SubsystemBase {

    private final DcMotorEx wrist;
    private final boolean continuousMode;
    private boolean zeroPositionMode = false;

    public static final float ANGLE_PER_SECOND = 60;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("WristSubsystem");

    public WristSubsystem(final DcMotorEx wrist, boolean continuousMode) {
        this.wrist = wrist;
        this.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.wrist.setDirection(DcMotorSimple.Direction.REVERSE);
        this.continuousMode = continuousMode;
        MatchLogger.getInstance().genericLog("Wrist", MatchLogger.FileType.WRIST, "TicksPerRev", wrist.getMotorType().getTicksPerRev(), "Gearing", wrist.getMotorType().getGearing());
    }

    public void zero() {
        DcMotor.RunMode previousMode = this.wrist.getMode();
        this.wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.wrist.setMode(previousMode);
        MatchLogger.getInstance().genericLog("Wrist", MatchLogger.FileType.WRIST, "Zeroed Wrist");
    }

    public void moveWrist(double frontward, double backward) {
        double power = frontward - backward;
        dbp.debug(String.format(Locale.ENGLISH, "Power: %f", power), true);
        dbp.debug("Servo position: " + getPosition(), true);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MatchLogger.getInstance().genericLog("Wrist", MatchLogger.FileType.WRIST, "Position", getPosition());
        setVelocity(power);
    }

    public void setWristPosition(int position) {
        setWristPosition(position, 1);
    }

    public void setWristPosition(int position, float speed) {
        int adjustedPosition = positionFromAngle(position);

        wrist.setTargetPosition(adjustedPosition);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setVelocity(speed);
        MatchLogger.getInstance().genericLog("Wrist", MatchLogger.FileType.WRIST, "Position", position, "Target Position", adjustedPosition, "Speed", speed);

        System.out.println("POSITION DATA:"+adjustedPosition+", "+position+", "+wrist.getMotorType().getTicksPerRev()+", "+wrist.getMotorType().getGearing());
    }

    private void setVelocity(double scale) {
        MatchLogger.getInstance().genericLog("Wrist", MatchLogger.FileType.WRIST, "Velocity", ANGLE_PER_SECOND*scale, "DEGREES");
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
