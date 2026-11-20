package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;


@TeleOp(name="Launcher Test", group="test")
@Config
public class LauncherTest extends LinearOpMode {

    private DcMotorEx launcher;
    private Servo feeder;
    private Servo limelightservo;
    private Limelight3A limelight;
    private DcMotor intake;

    public static PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(15,1,0,13);
    public static double LAUNCHER_TARGET_VEL = -1900;
    public static double LIMELIGHT_SERVO = 0.15;
    public static double INTAKE_SPEED = 0.4;

    private static boolean intakeActive = false;


    @Override
    public void runOpMode() {
        launcher  = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(Servo.class, "feeder");
        limelightservo = hardwareMap.get(Servo.class, "limelightservo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry = FtcDashboard.getInstance().getTelemetry();

        limelight.pipelineSwitch(3);
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            double[] py = null;

            LLResult result = limelight.getLatestResult();

            if (gamepad1.leftBumperWasPressed()) {
                intakeActive = !intakeActive;
            }
            if (intakeActive) {
                intake.setPower(INTAKE_SPEED);
            } else {
                intake.setPower(0);
            }

            if (result != null) {
                py = result.getPythonOutput();
                telemetry.addData("ty", py[2]);
            }


            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
            launcher.setVelocity(LAUNCHER_TARGET_VEL);
            limelightservo.setPosition(LIMELIGHT_SERVO);


            if (gamepad1.right_trigger >= 0.3) {
                feeder.setPosition(0.04);
            } else {
                feeder.setPosition(0.3);
            }
            telemetry.addData("Velocity", launcher.getVelocity());
            telemetry.update();
        }
    }
}