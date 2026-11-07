package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// Backup teleop that talks to the hardware directly so it still works if the main code breaks.
// Left stick: drive and strafe. Right stick: turn.
// Right trigger (hold): open the feeder gate. Right bumper: intake on/off.
// Left bumper (hold): run the flywheel backward to clear a stuck ball.
// Values below can be changed on FTC Dashboard under SimpleTeleOp.
@Config
@TeleOp(name = "Simple TeleOp", group = "Test")
public class SimpleTeleOp extends LinearOpMode {

    // flywheel speed is in ticks/sec
    public static double KP = 120, KI = 0.2, KD = 12, KF = 150;
    public static double LAUNCH_VEL = 1200;
    public static double REVERSE_POWER = 0.5;
    public static double FEED_OPEN = 0.04;
    public static double FEED_CLOSED = 0.3;
    public static double INTAKE_POWER = 0.8;

    @Override
    public void runOpMode() {
        // show telemetry on the Driver Hub and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // drive motors
        DcMotor fl = hardwareMap.get(DcMotor.class, "FL");
        DcMotor bl = hardwareMap.get(DcMotor.class, "BL");
        DcMotor fr = hardwareMap.get(DcMotor.class, "FR");
        DcMotor br = hardwareMap.get(DcMotor.class, "BR");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[]{fl, bl, fr, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // flywheel, intake and feeder gate
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo feeder = hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(FEED_CLOSED);

        boolean intakeOn = false;

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // mecanum drive, scaled down so no wheel goes over full power
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double flp = forward + strafe + turn;
            double blp = forward - strafe + turn;
            double frp = forward - strafe - turn;
            double brp = forward + strafe - turn;
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(flp), Math.abs(blp)),
                    Math.max(Math.abs(frp), Math.abs(brp))));
            fl.setPower(flp / max);
            bl.setPower(blp / max);
            fr.setPower(frp / max);
            br.setPower(brp / max);

            // reverse the flywheel while held, otherwise hold LAUNCH_VEL and feed on the trigger
            if (gamepad1.left_bumper) {
                launcher.setPower(-REVERSE_POWER);
                feeder.setPosition(FEED_CLOSED);
            } else {
                launcher.setVelocityPIDFCoefficients(KP, KI, KD, KF);
                launcher.setVelocity(LAUNCH_VEL);
                feeder.setPosition(gamepad1.right_trigger >= 0.3 ? FEED_OPEN : FEED_CLOSED);
            }

            // intake toggle
            if (gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
                intake.setPower(intakeOn ? INTAKE_POWER : 0);
            }

            telemetry.addData("Flywheel", "%.0f / %.0f", launcher.getVelocity(), LAUNCH_VEL);
            telemetry.addData("Intake", intakeOn ? "on" : "off");
            telemetry.update();
        }
    }
}
