package org.firstinspires.ftc.forteaching;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SwerveModule {
    private DcMotorEx motor;
    private CRServo servo;
    private Lamprey2Encoder encoder;
    private static final double INV_PI = 1.0 / Math.PI;
    private static final double angleError = 3;

    private void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
        }
    }

    private double calcError(double targetDegrees) {
        // We have to deal with the 'around the edge' checks
        double val = targetDegrees - this.getAngle();
        if (val > 180) {
            val -= 360;
        } else if (val < -180) {
            val += 360;
        }
        return val;
    }

    private double lastErr;
    private double lastInt;
    private ElapsedTime lastErrTime;
    private ElapsedTime totalTime;
    private ElapsedTime delayer;

    // Err is "# of degrees away from target"
    private double pidCalc(double err) {
        double timeDelta = lastErrTime.seconds();
        lastErrTime.reset();
        boolean early = (totalTime.milliseconds() < 100);
        double de_dt = early ? 0 : ((err - lastErr) / timeDelta); // derivative
        double e_t = early ? 0 : (lastInt + err * timeDelta); // integral
        double value = servoPid.p * err
                + servoPid.i * e_t
                + servoPid.d * de_dt
                + servoPid.f * Math.signum(err);
        err = lastErr;
        lastInt = e_t;
        return Math.max(-1.0, Math.min(1.0, value));
    }

    public static PIDFCoefficients servoPid = new PIDFCoefficients(4e-3, 6e-4, 1e-7, 0.15);

    public void resetPidData() {
        lastErr = 0;
        lastInt = 0;
        lastErrTime.reset();
        totalTime.reset();
    }

    public Pair<Double, Double> headToDegrees(double angle) {
        double err = calcError(angle);
        double val = pidCalc(err);
        servo.setPower(val);
        return new Pair<>(err, val);
    }

    public double getAngle() {
        return encoder.getAngle();
    }
    /*
    public void turnToRadians(double angle, Telemetry telemetry) {
        turnToDegrees(angle * 180 * INV_PI, telemetry);
    }
    */

    public void setRotatePower(double d) {
        servo.setPower(d);
    }

    public void setDrivePower(double d) {
        motor.setPower(d);
    }

    public SwerveModule(HardwareMap hw, String name) {
        motor = hw.get(DcMotorEx.class, name + "Motor");
        servo = hw.get(CRServo.class, name + "Servo");
        encoder = new Lamprey2Encoder(hw, name + "Encoder");
        lastErr = 0;
        lastInt = 0;
        lastErrTime = new ElapsedTime();
        totalTime = new ElapsedTime();
        delayer = new ElapsedTime();
    }

    public void manualDriveDegrees(double power, int angle) {
        // double servoError = goToServo(angle);
    }

    public void manualDriveRadians(double power, double angle) {
        manualDriveDegrees(power, (int) Math.round(angle * 180 / Math.PI));
    }
}
