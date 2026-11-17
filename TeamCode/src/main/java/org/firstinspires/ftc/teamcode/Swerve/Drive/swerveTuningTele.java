package org.firstinspires.ftc.teamcode.Swerve.Drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

/// todo dereference everything from this
@Config
@TeleOp
public class swerveTuningTele extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private HWMap hwMap;
    private SwerveDrivetrain swerveDrivetrain;

    public double x, y, heading;
    public double BotHeading;
    public boolean locked;

    GoBildaPinpointDriver odo;
    private double Xoffset, Yoffset;
    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    public static double xrate = 2.5, yrate = 2.5, headingrate = 2.2;

    public static double offsets[] = new double[]{-1.5, -0.8, -2.5, 1.1};
    public static boolean inverses[] = new boolean[]{false, false, false, false};
    public static double scalars[] = new double[]{-1, 1, -1, -1};

    public static int i;
    public static boolean gamepad;

    Logger logger;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        logger = new Logger(telemetry);

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);
        HeadingRate = new SlewRateLimiter(headingrate);

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
        swerveDrivetrain = new SwerveDrivetrain(hwMap, logger);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Initialize modules to current encoder positions to prevent startup spinning
        swerveDrivetrain.initializeModules();

        while (opModeIsActive()) {
            if (gamepad1.options) {
                odo.resetPosAndIMU();
                odo.update();
            }
            pos = odo.getPosition();
            BotHeading = -pos.getHeading(RADIANS);

            x = gamepad1.left_stick_x; //getting gamepad inputs
            y = -gamepad1.left_stick_y;
            heading = -gamepad1.right_stick_x;

            if (abs(x) < 0.02) {
                x = 0;
            }

            if (abs(y) < 0.02) {
                y = 0;
            }

            if (abs(heading) < 0.02) {
                heading = 0;
            }

//            if (x == 0 && y == 0 && heading == 0) {
//                locked = true;
//            }
//            else {
//                locked = false;
//            }

            Pose drive = new Pose((new Point(x, y).rotate(BotHeading)), heading);

            XRate.setPositiveRateLimit(xrate);
            XRate.setNegativeRateLimit(-xrate);

            YRate.setPositiveRateLimit(yrate);
            YRate.setNegativeRateLimit(-yrate);

            HeadingRate.setPositiveRateLimit(headingrate);
            HeadingRate.setNegativeRateLimit(-headingrate);

            drive = new Pose(XRate.calculate(drive.x), YRate.calculate(drive.y), HeadingRate.calculate(drive.heading));

            swerveDrivetrain.setOffsets(offsets);
            swerveDrivetrain.setInverses(inverses);

            swerveDrivetrain.setPose(drive, BotHeading, 12.4);
            swerveDrivetrain.updateModule(i);

            telemetry.addData("x", drive.x);
            telemetry.addData("y", drive.y);
            telemetry.addData("heading", drive.heading);
            telemetry.addData("locked", locked);
            swerveDrivetrain.log();
            telemetry.update();
        }
    }
}
