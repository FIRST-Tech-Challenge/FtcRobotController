package org.firstinspires.ftc.teamcode.Swerve.Drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.headingrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.inverses;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.offsets;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.scalars;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.xrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.yrate;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.HWMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;

@Config
@TeleOp
public class SwerveTest extends LinearOpMode {
    // todo make this the tuning tele
    private ElapsedTime runtime = new ElapsedTime();

    Logger logger;
    private HWMap hwMap;
    private SwerveDrivetrain swerveDrivetrain;

    public static double x, y, heading;
    public double BotHeading;
    public boolean locked;

    GoBildaPinpointDriver odo;
    private double Xoffset, Yoffset;
    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;

    public static double[] MotorScalars = new double[]{-1, 1, -1, -1};
    public static double[] Zeros = new double[]{-1.5, 0.5, -2.5, 1.1};

    public static double kgain = 2;

    public static double P, I, D, F;


    public static double lockdelay;

    @Override
    public void runOpMode() throws InterruptedException {
        P = 0.15;
        D = 0.005;
        kgain = 2;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        logger = new Logger(telemetry);

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);
        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Xoffset = 10.5;
        Yoffset = 1; //find in CAD
        odo.setOffsets(Xoffset, Yoffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        Pose2D startingpos = new Pose2D(DistanceUnit.CM, 0.0, 0.0, AngleUnit.RADIANS, 0.0);
        odo.setPosition(startingpos);
        odo.recalibrateIMU();

        hwMap = new HWMap(hardwareMap);
        swerveDrivetrain = new org.firstinspires.ftc.teamcode.Swerve.Drive.SwerveDrivetrain(hwMap, logger);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double voltage = hwMap.getVoltageSensor().getVoltage();

            swerveDrivetrain.setMotorScaling(MotorScalars);
            swerveDrivetrain.setOffsets(Zeros);
            swerveDrivetrain.setKgain(kgain);
            swerveDrivetrain.setHeadingControllerPIDF(P, I, D, F);
            swerveDrivetrain.setlockdelay(lockdelay);

            //swerveDrivetrain.calculateCurrentBasedScalers();
            //swerveDrivetrain.calculateVelocityBasedScalers();

            if (gamepad1.options) {
                odo.resetPosAndIMU();
            }

            odo.update();
            pos = odo.getPosition();
            BotHeading = -pos.getHeading(RADIANS);

            Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(-gamepad1.left_stick_x, gamepad1.left_stick_y))), (TurningScaler.Scale(gamepad1.right_stick_x, 0.01, 0.66, 4)));
            drive = new Pose(new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)).rotate(BotHeading), HeadingRate.calculate(drive.heading));

            if (abs(drive.x) < 0.02) {
                drive.x = 0;
            }

            if (abs(drive.y) < 0.02) {
                drive.y = 0;
            }

            if (abs(drive.heading) < 0.02) {
                drive.heading = 0;
            }

            if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                locked = true;
            } else {
                locked = false;
            }

            swerveDrivetrain.setPose(drive, BotHeading, voltage);
            swerveDrivetrain.setLocked(locked);
            swerveDrivetrain.updateModules();

            telemetry.addData("Bot Heading", BotHeading);
            swerveDrivetrain.log();
            telemetry.update();

        }

    }

}