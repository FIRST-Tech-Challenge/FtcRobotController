package org.firstinspires.ftc.teamcode.Swerve.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Core.Logger;


//this class is to find zeros, inverses,and check each module's coordinate system is in accordance
@Config
@TeleOp
public class SwerveCalibration extends LinearOpMode {

    // Front Left Module Hardware
    public DcMotorEx FLM;
    public CRServoImplEx FLS;
    public AnalogInput FLE;

    // Front Right Module Hardware
    public DcMotorEx FRM;
    public CRServoImplEx FRS;
    public AnalogInput FRE;

    // Back Right Module Hardware
    public DcMotorEx BRM;
    public CRServoImplEx BRS;
    public AnalogInput BRE;

    // Back Left Module Hardware
    public DcMotorEx BLM;
    public CRServoImplEx BLS;
    public AnalogInput BLE;

    public static double[] zeros = new double[]{0, 0, 0, 0};
    public static boolean[] inverses = new boolean[]{false, false, false, false};
    public static double motorpower;
    public static double P, I, D;
    public static double target;

    Logger logger;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        logger = new Logger(telemetry);
        FLM = hardwareMap.get(DcMotorEx.class, "FLM");
        FRM = hardwareMap.get(DcMotorEx.class, "FRM");
        BLM = hardwareMap.get(DcMotorEx.class, "BLM");
        BRM = hardwareMap.get(DcMotorEx.class, "BRM");

        FLS = hardwareMap.get(CRServoImplEx.class, "FLS");
        FRS = hardwareMap.get(CRServoImplEx.class, "FRS");
        BLS = hardwareMap.get(CRServoImplEx.class, "BLS");
        BRS = hardwareMap.get(CRServoImplEx.class, "BRS");

        FLE = hardwareMap.get(AnalogInput.class, "FLE");
        FRE = hardwareMap.get(AnalogInput.class, "FRE");
        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BRE = hardwareMap.get(AnalogInput.class, "BRE");

        SwerveModule frontLeftModule = new SwerveModule(FLM, FLS, FLE, 0.0, false, logger);
        SwerveModule frontRightModule = new SwerveModule(FRM, FRS, FRE, 0.0, false, logger);
        SwerveModule backRightModule = new SwerveModule(BRM, BRS, BRE, 0.0, false, logger);
        SwerveModule backLeftModule = new SwerveModule(BLM, BLS, BLE, 0.0, false, logger);

        int i = 0;

        SwerveModule[] modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Initialize modules to current encoder positions to prevent startup spinning
        for (SwerveModule m : modules) {
            m.initializePosition();
        }

        while (opModeIsActive()) {
            if (i == 4) {
                i = 0;
            }

            for (i = 0; i < 4; i++) {
                SwerveModule m = modules[i];
                m.setPID(P, I, D);
                m.setOffset(zeros[i]);
                m.setInverse(inverses[i]);
                m.update(target, motorpower);
            }

            //turning the module counterclockwise should increase the current rotation value
            //all modules must share the same coordinate system
            //all modules when a positive power is applied should go forward and vice versa

            telemetry.addData("frontLeftModule", frontLeftModule.getCurrentRotation());
            telemetry.addData("frontRightModule", frontRightModule.getCurrentRotation());
            telemetry.addData("backRightModule", backRightModule.getCurrentRotation());
            telemetry.addData("backLeftModule", backLeftModule.getCurrentRotation());
            telemetry.update();
        }
    }
}
