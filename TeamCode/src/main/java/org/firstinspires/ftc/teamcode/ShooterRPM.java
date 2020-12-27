package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Shooter RPM Control", group="Linear Opmode")
//@Disabled
public class ShooterRPM extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx shooterMotor = null;
    private final double MOTOR_RPM = 1150;
    private final double MOTOR_GEAR_RATIO = 1.0 / 5.2;
    private final double GEAR_RATIO = 1 / 1;
    private final double TICKS_PER_REVOLUTION = 145.6;
    private boolean yButtonPressed = false;
    private boolean aButtonPressed = false;
    private double targetMotorRPM = 500;
    private final double MIN_MOTOR_SPEED = 0.0;
    private final double MAX_MOTOR_SPEED = 1150;
    private final double RPM_INCREMENT = 50;
    private double powerSetPoint = 0.0;
    // Convert Ticks / ms to revolutions / minute (using time conversions from ms --> s --> min and also revolutions / ticks and also the gear ratio in denominator)
    private final double TICKS_PER_MS_TO_RPM = (1 * 1000 * 60) / (TICKS_PER_REVOLUTION * 1 * 1 * GEAR_RATIO);
    // Convert RPM (revolutions / min) to ticks / s (RPM x 1 min / 60s x ticks / rev)
    private final double RPM_TO_TICKS_PER_S = (1 * TICKS_PER_REVOLUTION) / (60 * 1);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        shooterMotor  = (DcMotorEx)hardwareMap.get(DcMotor.class, "Motor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPower(0.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double lastTime = runtime.milliseconds();
        double lastTicks = shooterMotor.getCurrentPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double newTime = runtime.milliseconds();
            double changeInTime = newTime - lastTime;
            lastTime = newTime;

            double newTicks = shooterMotor.getCurrentPosition();
            double changeInTicks = newTicks - lastTicks;
            lastTicks = newTicks;

            double currentRPM = 0;
            if (changeInTime > 0) {
                currentRPM = (changeInTicks * TICKS_PER_MS_TO_RPM) / (changeInTime);
            }

            if (!yButtonPressed && gamepad1.y) {
                //targetMotorRPM = targetMotorRPM + RPM_INCREMENT;
                targetMotorRPM = Range.clip(targetMotorRPM + RPM_INCREMENT, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
            } else if (!aButtonPressed && gamepad1.a) {
                //targetMotorRPM = targetMotorRPM - RPM_INCREMENT;
                targetMotorRPM = Range.clip(targetMotorRPM - RPM_INCREMENT, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
            }
            yButtonPressed = gamepad1.y;
            aButtonPressed = gamepad1.a;

            // Setup a variable for each drive wheel to save power level for telemetry
            double shooterPower = targetMotorRPM / MAX_MOTOR_SPEED;
            double shooterVelocity = targetMotorRPM * RPM_TO_TICKS_PER_S;

            double error = targetMotorRPM - currentRPM;
            double correction = 0;

            if (targetMotorRPM > 0) {
                correction = error / targetMotorRPM / 10.0;
            }

            shooterPower = Range.clip(shooterPower + correction, 0, 1.0);
            shooterVelocity = Range.clip(shooterVelocity + correction, 0, 1150 * RPM_TO_TICKS_PER_S);

            // Send calculated power to wheels
            double motorVelocity = shooterMotor.getVelocity();
            double motorVelocityRPM = shooterMotor.getVelocity() / RPM_TO_TICKS_PER_S;
            shooterMotor.setVelocity(shooterVelocity);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Shooter Power", "shooter (%.2f)", shooterPower);
            telemetry.addData("Shooter Velocity", "shooter (%.2f)", shooterVelocity);
            telemetry.addData("Current RPM", "%.03f rpm", currentRPM);
            telemetry.addData("Current Motor RPM", "%.03f rpm", motorVelocityRPM);
            telemetry.addData("Motor Controller TPS","%.03f tps", motorVelocity);
            telemetry.addData("Target RPM", "%.03f rpm", targetMotorRPM);
            telemetry.addData("Error", "%.03f rpm", error);
            telemetry.addData("Correction", "%.03f", correction);
            telemetry.update();
            idle();
        }
    }
}
