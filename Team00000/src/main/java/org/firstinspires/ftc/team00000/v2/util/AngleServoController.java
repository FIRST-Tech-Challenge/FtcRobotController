package org.firstinspires.ftc.team00000.v2.util;


import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AngleServoController {

    private final Servo servo;
    private final Telemetry tele;
    private final ExponentialMovingAverage ema =
            new ExponentialMovingAverage(AngleServoConfig.SMOOTHING_ALPHA);
    private double lastErrorDeg = 0;

    private double currentPosition = AngleServoConfig.SERVO_CENTER;
    private long   lastUpdateTime  = 0;

    public AngleServoController(Servo servo, Telemetry tele) {
        this.servo = servo;
        this.tele  = tele;
        servo.setPosition(currentPosition);
    }

    public void update(double angleErrorDeg) {

        long now   = System.currentTimeMillis();
        if (now - lastUpdateTime < AngleServoConfig.UPDATE_INTERVAL_MS) return;
        lastUpdateTime = now;

        // Smooth the error itself
        double smoothedErr = ema.update(angleErrorDeg);

        //Convert to absolute servo target
        double targetPos = AngleServoConfig.SERVO_CENTER
                - smoothedErr * AngleServoConfig.ANGLE_TO_POS_GAIN;
        targetPos = MathUtil.clamp(targetPos,
                AngleServoConfig.MIN_POS,
                AngleServoConfig.MAX_POS);

        // Move toward target with rate‑limit
        double delta = targetPos - currentPosition;

        if (Math.abs(delta) < AngleServoConfig.MOTION_THRESHOLD_POS) {
            pushTel(angleErrorDeg, smoothedErr, 0.0);
            return;                                       // close enough
        }

        delta = MathUtil.clamp(delta,
                -AngleServoConfig.MAX_DELTA_POS_PER_UPDATE,
                AngleServoConfig.MAX_DELTA_POS_PER_UPDATE);

        currentPosition += delta;
        servo.setPosition(currentPosition);

        pushTel(angleErrorDeg, smoothedErr, delta);
    }

    private void pushTel(double rawErr, double filtErr, double deltaPos) {
        if (tele == null) return;
        tele.addData("AngleErr",  "%.1f°", rawErr);
        tele.addData("FiltErr",   "%.1f°", filtErr);
        tele.addData("DeltaPos",  "%.3f",  deltaPos);
        tele.addData("ServoPos",  "%.3f",  currentPosition);
    }
}
