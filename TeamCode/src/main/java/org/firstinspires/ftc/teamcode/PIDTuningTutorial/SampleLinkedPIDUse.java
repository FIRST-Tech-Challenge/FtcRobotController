package org.firstinspires.ftc.teamcode.PIDTuningTutorial;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class SampleLinkedPIDUse extends LinearOpMode {
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0, 0, 0);

    // Copy your feedforward gains here
    public static double kV = 1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    private final PIDFController veloController = new PIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    @Override
    public void runOpMode() throws InterruptedException {
        // SETUP MOTORS //
        // Change my id
        DcMotorEx myMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        DcMotorEx myMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

        // Reverse as appropriate
        // myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that RUN_USING_ENCODER is not set
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Insert whatever other initialization stuff you do here

        waitForStart();

        if (isStopRequested()) return;

        // Start the veloTimer
        veloTimer.reset();

        while (!isStopRequested()) {
            ///// Run the velocity controller ////

            // Target velocity in ticks per second
            double targetVelo = 0.0;

            // Call necessary controller methods
            veloController.setTargetPosition(targetVelo);
            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();

            lastTargetVelo = targetVelo;

            // Get the velocity from the motor with the encoder
            double motorVelo = myMotor1.getVelocity();

            // Update the controller and set the power for each motor
            double power = veloController.update(motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);

            // Do your opmode stuff
        }
    }
}
