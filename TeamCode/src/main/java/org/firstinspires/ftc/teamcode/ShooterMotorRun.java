package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ShooterMotorRun extends OpMode {
    private DcMotorEx testshooter = null;
    private double currentPower = 0;
    private double currentPower2 = 0;

    // Replace 28.0 with your motor's exact Encoder Ticks Per Revolution (CPR)
    // Common FTC Motors:
    // GoBILDA Yellowjacket 6000 RPM: 28.0 ticks/rev
    // GoBILDA Yellowjacket 312 RPM: 537.7 ticks/rev
    // REV HD Hex Bare: 28.0 ticks/rev
    // Neverest Bare: 28.0 ticks/rev
    private final double TICKS_PER_REV = 28.0;

    @Override
    public void init() {
        testshooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        testshooter.setDirection(DcMotorSimple.Direction.FORWARD);
        testshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            currentPower += 0.1;

            if (currentPower > 1.0) {
                currentPower = 0.0;
            }
        }

        testshooter.setPower(currentPower);

        // Calculate actual RPM: (ticks/sec) / (ticks/rev) * 60 sec/min
        double currentTicksPerSec = testshooter.getVelocity();
        double currentRpm = (currentTicksPerSec / TICKS_PER_REV) * 60.0;

        // Telemetry outputs
        telemetry.addData("Set Power", "%.2f", currentPower);
        telemetry.addData("Target Velocity (ticks/s)", currentTicksPerSec);
        telemetry.addData("Calculated Speed (RPM)", "%.1f RPM", currentRpm);
        telemetry.update();
    }
}