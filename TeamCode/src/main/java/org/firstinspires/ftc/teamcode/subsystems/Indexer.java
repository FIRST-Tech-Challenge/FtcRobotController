package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Indexer {
    private static final String FEEDER_MOTOR_NAME       = "FeederMotor"; // port 2
    private static final String SPINDEXER_SERVO_NAME    = "SpindexerServo";
    private static final double SHOOTER_RPM_THRESHOLD   = 500.0; // 1/3 of 6000 RPM peak
    // Time for one 72-degree advance at ~60 RPM CRServo no-load. Calibrate physically.
    private static final double FEED_PULSE_SEC          = 0.315;

    private final DcMotor    feederMotor;
    private final CRServo    spindexerServo;

    private boolean           feeding  = false;
    private final ElapsedTime feedTimer = new ElapsedTime();

    public Indexer(HardwareMap hardwareMap) {
        feederMotor    = hardwareMap.get(DcMotor.class, FEEDER_MOTOR_NAME);
        spindexerServo = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_NAME);

        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // shooterRPM: live RPM from the shooter motor encoder.
    // fire: one-shot signal from Shooter.consumeFire() - true for exactly one loop when ready.
    // manualFire: one-shot signal (button B) - runs spindexer pulse regardless of RPM.
    public void update(double shooterRPM, boolean fire, boolean manualFire) {
        boolean aboveThreshold = shooterRPM >= SHOOTER_RPM_THRESHOLD;
        feederMotor.setPower(aboveThreshold ? 1.0 : 0.0);

        if (!feeding && ((fire && aboveThreshold) || manualFire)) {
            spindexerServo.setPower(-1.0);
            feedTimer.reset();
            feeding = true;
        }

        if (feeding && feedTimer.seconds() >= FEED_PULSE_SEC) {
            spindexerServo.setPower(0);
            feeding = false;
        }
    }

    public void stop() {
        feederMotor.setPower(0);
        spindexerServo.setPower(0);
    }
}
