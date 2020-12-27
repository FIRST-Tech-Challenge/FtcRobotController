package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@Disabled
public class ShooterControls {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx shootRight = null;
    private DcMotorEx shootLeft = null;
    private final double MOTOR_RPM = 1780;
    private final double EXTERNAL_GEAR_RATIO = 3.0 / 1.0;
    // private final double INTERNAL_GEAR_RATIO = 1.0 / 3.7;
    private final double TICKS_PER_REVOLUTION = 103.6;
    private boolean yButtonPressed = false;
    private boolean aButtonPressed = false;
    private double targetMotorRPM = 500;
    private final double MIN_MOTOR_SPEED = 0.0;
    private final double MAX_MOTOR_SPEED = 1780;
    private final double RPM_INCREMENT = 50;
    private double powerSetPoint = 0.0;
    // Convert Ticks / ms to revolutions / minute (using time conversions from ms --> s --> min and also revolutions / ticks and also the gear ratio in denominator)
    private final double TICKS_PER_MS_TO_RPM = (1 * 1000 * 60) / (TICKS_PER_REVOLUTION * 1 * 1 * EXTERNAL_GEAR_RATIO);
    // Convert RPM (revolutions / min) to ticks / s (RPM x 1 min / 60s x ticks / rev)
    private final double RPM_TO_TICKS_PER_S = (1 * TICKS_PER_REVOLUTION) / (60 * 1 * EXTERNAL_GEAR_RATIO);

    public void initialize(HardwareMap hw){
        shootRight = (DcMotorEx)hw.get(DcMotor.class, "ShootRgt");
        shootLeft = (DcMotorEx)hw.get(DcMotor.class, "ShootLft");
        shootRight.setDirection(DcMotor.Direction.FORWARD);
        shootLeft.setDirection(DcMotor.Direction.FORWARD);
        shootRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootRight.setPower(0.0);
        shootLeft.setPower(0.0);
    }

    public void whileActive(LinearOpMode op) {

        runtime.reset();
        double lastTime = runtime.milliseconds();
        double lastTicksRight = shootRight.getCurrentPosition();
        double lastTicksLeft = shootLeft.getCurrentPosition();

        // run until the end of the match (driver presses STOP)
        if (op.opModeIsActive()) {

            double newTime = runtime.milliseconds();
            double changeInTime = newTime - lastTime;
            lastTime = newTime;

            double newTicksRight = shootRight.getCurrentPosition();
            double changeInTicksRight = newTicksRight - lastTicksRight;
            lastTicksRight = newTicksRight;

            double newTicksLeft = shootLeft.getCurrentPosition();
            double changeInTicksLeft = newTicksLeft - lastTicksLeft;
            lastTicksLeft = newTicksLeft;

            double currentRPM = 0;
            if (changeInTime > 0) {
                currentRPM = (changeInTicksLeft * TICKS_PER_MS_TO_RPM) / (changeInTime);
            }

            if (!yButtonPressed && op.gamepad1.y) {
                //targetMotorRPM = targetMotorRPM + RPM_INCREMENT;
                targetMotorRPM = Range.clip(targetMotorRPM + RPM_INCREMENT, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
            } else if (!aButtonPressed && op.gamepad1.a) {
                //targetMotorRPM = targetMotorRPM - RPM_INCREMENT;
                targetMotorRPM = Range.clip(targetMotorRPM - RPM_INCREMENT, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
            }
            yButtonPressed = op.gamepad1.y;
            aButtonPressed = op.gamepad1.a;

            // Setup a variable for each drive wheel to save power level for telemetry
            double shooterPower = targetMotorRPM / MAX_MOTOR_SPEED;
            double shooterVelocity = targetMotorRPM * RPM_TO_TICKS_PER_S;

            double error = targetMotorRPM - currentRPM;
            double correction = 0;

            if (targetMotorRPM > 0) {
                correction = error / targetMotorRPM / 10.0;
            }

            shooterPower = Range.clip(shooterPower + correction, 0, 1.0);
            shooterVelocity = Range.clip(shooterVelocity + correction, 0, MOTOR_RPM * RPM_TO_TICKS_PER_S);

            // Send calculated power to wheels
            double motorVelocity = shootLeft.getVelocity();
            double motorVelocityRPM = shootLeft.getVelocity() / RPM_TO_TICKS_PER_S;
            shootLeft.setVelocity(shooterVelocity);
            shootRight.setVelocity(shooterVelocity);

            // Show the elapsed game time and wheel power.
            op.telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Shooter Power", "shooter (%.2f)", shooterPower);
            op.telemetry.addData("Shooter Velocity", "shooter (%.2f)", shooterVelocity);
            op.telemetry.addData("Current RPM", "%.03f rpm", currentRPM);
            op.telemetry.addData("Current Motor RPM", "%.03f rpm", motorVelocityRPM);
            op.telemetry.addData("Motor Controller TPS","%.03f tps", motorVelocity);
            op.telemetry.addData("Target RPM", "%.03f rpm", targetMotorRPM);
            op.telemetry.addData("Error", "%.03f rpm", error);
            op.telemetry.addData("Correction", "%.03f", correction);
        }
    }
}