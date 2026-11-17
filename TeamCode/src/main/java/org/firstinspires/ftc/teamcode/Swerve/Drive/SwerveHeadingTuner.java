package org.firstinspires.ftc.teamcode.Swerve.Drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.inverses;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.offsets;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.scalars;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.xrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.yrate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;


@Config
@TeleOp(name="Swerve: PD Heading Tuner (Synced)")
public class SwerveHeadingTuner extends LinearOpMode {

    public static double TEST_POWER = 0.6;
    public static long STEP_MS = 1000;

    public static double P_ADJUST = 0.02;
    public static double D_ADJUST = 0.002;
    public static double D_SCALING = 0.05;

    private SwerveDrivetrain swerveDrivetrain;
    private com.qualcomm.hardware.gobilda.GoBildaPinpointDriver odo;
    private SlewRateLimiter XRate, YRate;
    private HWMap hwMap;
    private final ElapsedTime timer = new ElapsedTime();

    private double finalP = 0.1;
    private double finalD = 0.0;
    private double bestDrift = Double.MAX_VALUE;
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Logger logger = new Logger(telemetry);
        hwMap = new HWMap(hardwareMap);

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);

        odo = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(10.5, 1.0, DistanceUnit.MM);
        odo.setEncoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD, com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        swerveDrivetrain = new SwerveDrivetrain(hwMap, logger);
        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        finalP = swerveDrivetrain.getP();
        finalD = swerveDrivetrain.getD();

        telemetry.addData("Status", "Synced with Drivetrain PID");
        telemetry.addData("Starting P", finalP);
        telemetry.update();

        waitForStart();

        // Initialize modules to current encoder positions to prevent startup spinning
        swerveDrivetrain.initializeModules();

        runMovementBatch("Forward/Backward", new Point(TEST_POWER, 0), 25);
        runMovementBatch("Side to Side Strafing", new Point(0, TEST_POWER), 25);
        runMovementBatch("Diagonal Strafing", new Point(TEST_POWER, TEST_POWER), 25);
        runMovementBatch("Other Diagonal Strafing", new Point(-TEST_POWER, TEST_POWER), 25);

        while (opModeIsActive()) {
            telemetry.addLine("--- REFINEMENT COMPLETE ---");
            telemetry.addData("Optimized P", finalP);
            telemetry.addData("Optimized D", finalD);
            telemetry.update();
        }
    }

    private void runMovementBatch(String batchName, Point move, int cycles) {
        for (int i = 0; i < cycles && opModeIsActive(); i++) {
            double direction = (i % 2 == 0) ? 1.0 : -1.0;
            double cycleDriftAccumulator = 0;
            int cycleSamples = 0;
            timer.reset();

            while (timer.milliseconds() < STEP_MS && opModeIsActive()) {
                odo.update();
                double botHeading = -odo.getPosition().getHeading(RADIANS);
                double error = normalizeRadians(0 - botHeading);

                cycleDriftAccumulator += Math.abs(error);
                cycleSamples++;

                double dt = 0.03;
                double errorRate = Math.abs(error - lastError) / dt;
                double dynamicD = finalD + (errorRate * D_SCALING);

                swerveDrivetrain.setHeadingControllerPIDF(finalP, 0, dynamicD, 0);

                double limX = XRate.calculate(move.x) * direction;
                double limY = YRate.calculate(move.y) * direction;
                Point rotated = new Point(limX, limY).rotate(botHeading);

                swerveDrivetrain.setPose(new Pose(rotated, 0), botHeading, hwMap.getVoltageSensor().getVoltage());
                swerveDrivetrain.updateModules();

                lastError = error;
            }

            stopRobot();
            double avgCycleDrift = cycleDriftAccumulator / cycleSamples;

            if (avgCycleDrift < bestDrift) {
                bestDrift = avgCycleDrift;
                finalP += P_ADJUST;
                finalD += D_ADJUST;
            } else {
                finalP -= P_ADJUST;
                finalD += D_ADJUST;
                P_ADJUST *= 0.95;
            }

            sendTelemetry(batchName, avgCycleDrift, i);
            sleep(150);
        }
    }

    private void stopRobot() {
        swerveDrivetrain.setPose(new Pose(0,0,0), 0.0, 12.0);
        swerveDrivetrain.updateModules();
    }

    private void sendTelemetry(String test, double drift, int cycle) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Phase", test);
        packet.put("Cycle", cycle);
        packet.put("Avg Drift", drift);
        packet.put("P", finalP);
        packet.put("D", finalD);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}