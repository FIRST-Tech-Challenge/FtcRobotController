

package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Supa dupa testing teleopmode")
@Config
public class SuperTestTeleop extends LinearOpMode {

    private DcMotorEx launcher;
    private DcMotor intake;
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private Servo feeder;

    public static PIDFCoefficients Pidf = new PIDFCoefficients(120,0.2,12,150);

    public static double vel = 1200;


    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;


    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        boolean right_trigger_block;
        float axial;
        float lateral;
        float yaw;
        double max;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        feeder = hardwareMap.get(Servo.class, "feeder");

        runtime = new ElapsedTime();
        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        launcher.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        feeder.setPosition(0.04);
        right_trigger_block = false;
        waitForStart();
        runtime.reset();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,Pidf);
            ((DcMotorEx) launcher).setVelocity(vel);
            if (gamepad1.right_trigger >= 0.3) {
                feeder.setPosition(0.04);
            } else {
                feeder.setPosition(0.3);
            }
            if (gamepad1.dpad_down) {
                feeder.setPosition(Math.min(Math.max(feeder.getPosition() - 0.01, 0), 1));
            }
            if (gamepad1.dpad_up) {
                feeder.setPosition(Math.min(Math.max(feeder.getPosition() + 0.01, 0), 1));
            }
            telemetry.addData("Feeder Servo:", feeder.getPosition());
            if (gamepad1.rightBumperWasPressed()) {
                if (intake.getPower() != 0) {
                    intake.setPower(0);
                } else {
                    intake.setPower(0.8);
                }
            }
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;
            frontLeftPower = axial + lateral + yaw;
            frontRightPower = (axial - lateral) - yaw;
            backLeftPower = (axial - lateral) + yaw;
            backRightPower = (axial + lateral) - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (max > 1) {
                frontLeftPower = frontLeftPower / max;
                frontRightPower = frontRightPower / max;
                backLeftPower = backLeftPower / max;
                backRightPower = backRightPower / max;
            }
            // Send calculated power to wheels.
            FL.setPower(frontLeftPower);
            FR.setPower(frontRightPower);
            BL.setPower(backLeftPower);
            BR.setPower(backRightPower);
            // Show the elapsed game time and wheel power.

            telemetry.addData("Velocity",launcher.getVelocity());
            telemetry.addData("Current", launcher.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }

}