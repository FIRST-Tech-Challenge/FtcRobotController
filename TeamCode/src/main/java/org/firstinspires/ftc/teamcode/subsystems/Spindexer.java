package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Spindexer {
    private static final String FLYWHEEL_MOTOR_NAME     = "SpindexerMotor"; // port 2
    private static final String FEEDER_SERVO_NAME       = "SpindexerServo";
    private static final double SPINDEXER_RPM_THRESHOLD = 2000.0; // 1/3 of 6000 RPM peak
    // Time for one 72-degree advance at ~60 RPM CRServo no-load. Calibrate physically.
    private static final double FEED_PULSE_SEC          = 0.20;

    private final DcMotor    flywheelMotor;
    private final CRServo    feederServo;

    private boolean           feeding  = false;
    private final ElapsedTime feedTimer = new ElapsedTime();

    public Spindexer(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotor.class, FLYWHEEL_MOTOR_NAME);
        feederServo   = hardwareMap.get(CRServo.class, FEEDER_SERVO_NAME);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // shooterRPM: live RPM from the turret motor encoder.
    // fire: one-shot signal from Turret.consumeFire() — true for exactly one loop when ready.
    public void update(double shooterRPM, boolean fire) {
        if (shooterRPM < SPINDEXER_RPM_THRESHOLD) {
            stop();
            feeding = false;
            return;
        }

        flywheelMotor.setPower(1.0);

        if (fire && !feeding) {
            feederServo.setPower(1.0);
            feedTimer.reset();
            feeding = true;
        }

        if (feeding && feedTimer.seconds() >= FEED_PULSE_SEC) {
            feederServo.setPower(0);
            feeding = false;
        }
    }

    public void stop() {
        flywheelMotor.setPower(0);
        feederServo.setPower(0);
    }
}
