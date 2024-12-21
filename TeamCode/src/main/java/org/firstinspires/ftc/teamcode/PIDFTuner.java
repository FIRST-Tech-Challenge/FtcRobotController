package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "PIDFTuner")
@Config
public class PIDFTuner extends OpMode {
    private AutonomousHandler autoHandler;
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;
    DcMotor cap;
    private ArmPIDFController pidf;
    public static FtcDashboard dashboard;
    public static double ProportionalTerm = 0;
    public static double IntegralTerm = 0;
    public static double DerivativeTerm = 0;
    public static double FeedforwardTerm = 0;
    public static double ticsPerDegree = 100;
    public static double reference = (double) 590 /90;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();;
        // Motors & Sensors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor spindle = hardwareMap.get(DcMotor.class, "spindle");
        SparkFunOTOS odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
        cap = hardwareMap.get(DcMotor.class, "cap");
        // Servos
        LSLower = hardwareMap.get(ServoImplEx.class, "LSLower");
        LSTop = hardwareMap.get(ServoImplEx.class, "LSTop");
        RevTouchSensor lswitch = hardwareMap.get(RevTouchSensor.class, "Lswitch");

        // Define Servo range
        LSLower.setPwmEnable();
        LSTop.setPwmEnable();
        LSLower.scaleRange(0, 1);
        LSTop.scaleRange(0, 1);
        LSLower.setPosition(0.1);
        LSTop.setPosition(0.1);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Breaking mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf = new ArmPIDFController(ProportionalTerm, IntegralTerm, DerivativeTerm, FeedforwardTerm);
    }

    @Override
    public void loop() {
        telemetry.addData("capstan", cap.getCurrentPosition());
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("capstanValue", cap.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
        pidf.resetPIDF(ProportionalTerm, IntegralTerm, DerivativeTerm, FeedforwardTerm);
        cap.setPower(pidf.getOutput(cap.getCurrentPosition(), reference, (1/(ticsPerDegree*cap.getCurrentPosition()))));
    }
}