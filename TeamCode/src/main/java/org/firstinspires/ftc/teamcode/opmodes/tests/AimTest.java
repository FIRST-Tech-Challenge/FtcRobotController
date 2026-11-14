

package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@TeleOp(name = "AimTest", group = "Test")
@Config
public class AimTest extends LinearOpMode {

    private DcMotorEx launcher;
    private DcMotor intake;
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private Servo feeder;
    private Limelight3A limelight;

    public static PIDFCoefficients Pidf = new PIDFCoefficients(120,0.67,12,150);

    public static double vel = 1150;


    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    boolean intakeActive = false;


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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        feeder = hardwareMap.get(Servo.class, "feeder");

        runtime = new ElapsedTime();
        launcher.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        feeder.setPosition(0.04);
        limelight.pipelineSwitch(0);
        limelight.start();
        dashboard.startCameraStream(limelight, 10);
        LLResult result = limelight.getLatestResult();
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
                intakeActive = !intakeActive;
            }
            if (intakeActive) {
                intake.setPower(0.8);
            } else {
                intake.setPower(0);
            }
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            result = limelight.getLatestResult();
            //aiming
            if ((gamepad1.left_trigger > 0.3) && (result.isValid())) {
                double RSaimYaw = 0, RAaimYaw = 0, BSaimYaw = 0, BAaimYaw = 0;
                double RSaimAxial = 0, RAaimAxial = 0, BSaimAxial = 0, BAaimAxial = 0;
                int RS = 0, RA = 0, BS = 0, BA = 0;
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                double i;
                for (LLResultTypes.FiducialResult fr : fiducialResults) {

                    if ((fr.getFiducialId() >= 30) && (fr.getFiducialId() <= 33)) {
                        RS++;
                        RSaimYaw += fr.getTargetXDegrees();
                        RSaimAxial += fr.getTargetYDegrees();
                    }
                    if ((fr.getFiducialId() >= 34) && (fr.getFiducialId() <= 37)) {
                        RA++;
                        RAaimYaw += fr.getTargetXDegrees();
                        RAaimAxial += fr.getTargetYDegrees();
                    }
                    if ((fr.getFiducialId() >= 38) && (fr.getFiducialId() <= 41)) {
                        BS++;
                        BSaimYaw += fr.getTargetXDegrees();
                        BSaimAxial += fr.getTargetYDegrees();
                    }
                    if ((fr.getFiducialId() >= 42) && (fr.getFiducialId() <= 45)) {
                        BA++;
                        BAaimYaw += fr.getTargetXDegrees();
                        BAaimAxial += fr.getTargetYDegrees();
                    }
                }
                //determine highest
                int[] choices = {RS,RA,BS,BA};
                double[] choiceyaw = {RSaimYaw,RAaimYaw,BSaimYaw,BAaimYaw};
                double[] choiceAxial = {RSaimAxial,RAaimAxial,BSaimAxial,BAaimAxial};

                int highest = choices[0];
                double closestToZero = choiceyaw[0];
                int selectedCluster = 0;

                for (int v=1; v < choices.length; v++) {
                    if (choices[v] > highest || (choices[v] == highest && Math.abs(choiceyaw[v]) < Math.abs(closestToZero))) {
                        highest = choices[v];
                        closestToZero = choiceyaw[v];
                        selectedCluster = v;
                    }

                }

                double aimYaw = choiceyaw[selectedCluster];
                double aimAxial = choiceAxial[selectedCluster];

                //divide by chosen clusters amount of tags to find average (except if zero)
                if (choices[selectedCluster] != 0) {
                    aimYaw = aimYaw / choices[selectedCluster];
                    aimAxial = aimAxial / choices[selectedCluster];
                    if (gamepad1.left_bumper) {
                        axial = (float) aimAxial;
                        yaw = (float) -aimYaw;
                    }
                    telemetry.addData("AimYaw", aimYaw);
                    telemetry.addData("AimAxial", aimAxial);
                } else {
                    telemetry.addData("Error", "Divide by zero");
                }
            }


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
            telemetry.addData("Intake", intakeActive);
            telemetry.update();
        }
    }

}