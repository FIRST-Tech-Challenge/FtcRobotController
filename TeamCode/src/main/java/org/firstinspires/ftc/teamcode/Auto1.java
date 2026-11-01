package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Auto1: shoots the 4 pre-loaded balls and nothing else. The robot does not move.
 * It waits 15 seconds after START (the flywheel speeds up to 3000 RPM meanwhile), then the
 * siphon turns 3 times forward, half a turn backward, and 2 times forward again.
 * The siphon servo can't count its turns, so each turn lasts SECONDS_PER_TURN: time one siphon
 * turn at SIPHON_POWER and put that number here.
 */
@Autonomous(name = "Auto1", group = "Robot", preselectTeleOp = "TeleOp")
public class Auto1 extends LinearOpMode {

    private static final double RPM = 3000.0;
    private static final double WAIT_SECONDS = 15.0;     // waits this long after START before shooting
    private static final double SECONDS_PER_TURN = 2.0;  // measure on the robot
    private static final double SIPHON_POWER = 0.5;      // half speed: steadier shots
    private static final double TICKS_PER_REV = 28.0;    // flywheel motor, same as TeleOp

    @Override
    public void runOpMode() {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "turret");
        CRServo siphon = hardwareMap.get(CRServo.class, "servosiphon");
        DcMotor intake = hardwareMap.get(DcMotor.class, "mainmotorintake");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftfeeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightfeeder");
        siphon.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(40.0, 0.0, 0.0, 32767.0 / (6000.0 * TICKS_PER_REV / 60.0));

        waitForStart();
        flywheel.setVelocity(RPM * TICKS_PER_REV / 60.0);  // speeds up during the wait

        // Wait 15 seconds.
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < WAIT_SECONDS) {
            telemetry.addData("Waiting", "%.1f s until shooting", WAIT_SECONDS - timer.seconds());
            telemetry.addData("Flywheel", "%.0f RPM", rpm(flywheel));
            telemetry.update();
        }

        // Shoot: the intake and feeders push the balls up to the siphon the whole time.
        intake.setPower(1.0);
        leftFeeder.setPower(1.0);
        rightFeeder.setPower(1.0);
        spinSiphon(siphon, flywheel, 3);
        spinSiphon(siphon, flywheel, -0.5);
        spinSiphon(siphon, flywheel, 2);
        // Done. Everything stops when the OpMode ends.
    }

    /** Turns the siphon that many times: positive = forward (shooting), negative = backward. */
    private void spinSiphon(CRServo siphon, DcMotorEx flywheel, double turns) {
        siphon.setPower(Math.signum(turns) * SIPHON_POWER);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < Math.abs(turns) * SECONDS_PER_TURN) {
            telemetry.addData("Siphon", "%s, about %.1f of %.1f turns", turns > 0 ? "forward" : "backward",
                    timer.seconds() / SECONDS_PER_TURN, Math.abs(turns));
            telemetry.addData("Flywheel", "%.0f RPM", rpm(flywheel));
            telemetry.update();
        }
        siphon.setPower(0);
    }

    private static double rpm(DcMotorEx flywheel) {
        return flywheel.getVelocity() * 60.0 / TICKS_PER_REV;
    }
}
