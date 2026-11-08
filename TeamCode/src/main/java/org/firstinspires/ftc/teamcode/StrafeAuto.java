package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Strafes right ~1 foot, then strafes left ~1 foot back to the start, at 0.4 power.
 * Uses the drive motor encoders (RUN_TO_POSITION).
 */
@Autonomous(name = "Strafe Auto", group = "Auto")
public class StrafeAuto extends LinearOpMode {

    // ---- Tune these for your robot ----
    // goBILDA 5203 312 RPM motor = 537.7 ticks/rev. (435 RPM = 384.5, REV HD 20:1 = 560)
    static final double TICKS_PER_REV     = 537.7;
    static final double GEAR_RATIO        = 1.0;     // wheel turns per motor turn (external gearing)
    static final double WHEEL_DIAMETER_IN = 4.094;   // goBILDA 104mm mecanum
    // Mecanum wheels slip sideways, so strafing needs more ticks than driving straight.
    // If the robot strafes short, raise this; if it overshoots, lower it.
    static final double STRAFE_MULTIPLIER = 1.1;

    static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_IN * Math.PI);

    static final double SPEED = 0.4;
    static final double DISTANCE_IN = 12.0;
    // -----------------------------------

    private DcMotor fl, fr, bl, br;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // Must match MecanumTeleOp
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        strafe(DISTANCE_IN, SPEED);   // right 1 ft
        sleep(250);
        strafe(-DISTANCE_IN, SPEED);  // left 1 ft (back to start)

        telemetry.addLine("Done");
        telemetry.update();
    }

    /** Positive inches = strafe right, negative = strafe left. */
    private void strafe(double inches, double speed) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH * STRAFE_MULTIPLIER);

        // Strafe right: fl +, br +, fr -, bl -
        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        br.setTargetPosition(br.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() - ticks);
        bl.setTargetPosition(bl.getCurrentPosition() - ticks);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(speed));
        }

        while (opModeIsActive()
                && (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            telemetry.addData("Target ticks", ticks);
            telemetry.addData("fl / fr", "%d / %d", fl.getCurrentPosition(), fr.getCurrentPosition());
            telemetry.addData("bl / br", "%d / %d", bl.getCurrentPosition(), br.getCurrentPosition());
            telemetry.update();
        }

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
