package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;

@Config
public class SwerveDrivetrain {
    public SwerveModule leftFront, leftBack, rightFront, rightBack;
    public SwerveModule[] swerveModules;
    private DcMotorEx mrightFront;
    private DcMotorEx mleftFront;
    private DcMotorEx mleftBack;
    private DcMotorEx mrightBack;

    private CRServo srightFront;
    private CRServo sleftFront;
    private CRServo sleftBack;
    private CRServo srightBack;

    private AnalogInput erightFront;
    private AnalogInput eleftFront;
    private AnalogInput eleftBack;
    private AnalogInput erightBack;
    private static double analogRangeRightFront = 3.3;
    private static double analogRangeLeftFront = 3.3;
    private static double analogRangeLeftBack = 3.3;
    private static double analogRangeRightBack = 3.3;


    public static final double E_FRONT_RIGHT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_FRONT_LEFT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_BACK_LEFT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_BACK_RIGHT_OFFSET = 1.1;//myArbitraryRadValue

    public final double TRACKWIDTH = 12.6378;
    public final double WHEELBASE = 12.6378;
    private final double R = Math.hypot(TRACKWIDTH, WHEELBASE);
    public static double leftFrontOffset = 0, leftBackOffset = 0, rightFrontOffset = 0, rightBackOffset = 0;
    double[] ws;
    double[] wa;
    double max = 0.0;

    public SwerveDrivetrain() {
        leftFront = new SwerveModule(hardwareMap.get(DcMotorEx.class, "frontLeft"), hardwareMap.get(CRServo.class, "sfrontLeft"), new AbsoluteAnalogEncoder(eleftFront, analogRangeLeftFront).zero(E_FRONT_LEFT_OFFSET).setInverted(true));
        leftBack = new SwerveModule(hardwareMap.get(DcMotorEx.class, "backLeft"), hardwareMap.get(CRServo.class, "sbackLeft"), new AbsoluteAnalogEncoder(eleftBack, analogRangeLeftBack).zero(E_BACK_LEFT_OFFSET).setInverted(true));
        rightFront = new SwerveModule(hardwareMap.get(DcMotorEx.class, "frontRight"), hardwareMap.get(CRServo.class, "sfrontRight"), new AbsoluteAnalogEncoder(erightFront, analogRangeRightFront).zero(E_FRONT_RIGHT_OFFSET).setInverted(true));
        rightBack = new SwerveModule(hardwareMap.get(DcMotorEx.class, "backRight"), hardwareMap.get(CRServo.class, "sbackRight"), new AbsoluteAnalogEncoder(erightBack, analogRangeRightBack).zero(E_BACK_RIGHT_OFFSET).setInverted(true));

        swerveModules = new SwerveModule[]{leftFront, leftBack, rightFront, rightBack};
//        for (SwerveModule m : swerveModules) m.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void read() {
        for (SwerveModule module : swerveModules) module.read();
    }

//    @Override
    public void set(Pose pose) {
        double x = pose.x;
        double y = pose.y;
        double head = pose.heading;

        double a = x - head * (WHEELBASE / R),
                b = x + head * (WHEELBASE / R),
                c = y - head * (TRACKWIDTH / R),
                d = y + head * (TRACKWIDTH / R);

        ws = new double[]{Math.hypot(b, c), Math.hypot(b, d), Math.hypot(a, d), Math.hypot(a, c)};
        wa = new double[]{Math.atan2(b, c), Math.atan2(b, d), Math.atan2(a, d), Math.atan2(a, c)};
    }


    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = swerveModules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]) + 0.1 * Math.signum(ws[i]));
            m.setTargetRotation((wa[i]) % (2*Math.PI));
        }
    }

    public String getTelemetry() {
        return leftFront.getTelemetry("leftFrontModule") + "\n" +
                leftBack.getTelemetry("leftRearModule") + "\n" +
                rightFront.getTelemetry("rightFrontModule") + "\n" +
                rightBack.getTelemetry("rightRearModule") + "\n";
    }
}