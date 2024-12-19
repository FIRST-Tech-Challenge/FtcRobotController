package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDtuning extends OpMode {

    private PIDController LiftController;
    private PIDController ExtendController;

    public static double Lp = 0.03, Li = 0, Ld = 0.0005;
    public static double Ep = .01, Ei = 0, Ed = .0004;
    public static double Lf = 0.037;
    public static double Ef = 0;

    public static int LiftTarget = 0;
    public static int ExtendTarget = 0;

    private final double lift_ticks_in_degrees = 1.068055;
    private final double extend_ticks_in_degrees = .403;

    private DcMotorEx LeftLift;
    private DcMotorEx RightLift;
    private DcMotorEx IntakeLeft;
    private DcMotorEx IntakeRight;

    @Override
    public void init(){
        LiftController = new PIDController(Lp, Li, Ld);
        ExtendController = new PIDController(Ep, Ei, Ed);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LeftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");
        RightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");
        IntakeLeft = hardwareMap.get(DcMotorEx.class, "Intake Left");
        IntakeRight = hardwareMap.get(DcMotorEx.class, "Intake Right");


        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses the direction the motor turns

        LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns
    }

    @Override
    public void loop(){
        LiftController.setPID(Lp, Li, Ld);
        ExtendController.setPID(Ep, Ei, Ed);
        int LiftPos = LeftLift.getCurrentPosition();
        int ExtendPos = IntakeLeft.getCurrentPosition();
        double Lpid = LiftController.calculate(LiftPos, LiftTarget);
        double Epid = ExtendController.calculate(ExtendPos, ExtendTarget);
        double LiftFF = Math.cos(Math.toRadians(LiftTarget / lift_ticks_in_degrees)) * Lf;
        double ExtendFF = Math.cos(Math.toRadians(ExtendTarget / extend_ticks_in_degrees)) * Ef;

        double LiftPower = Lpid + LiftFF;
        double ExtendPower = Epid + ExtendFF;

        LeftLift.setPower(LiftPower);
        RightLift.setPower(LiftPower);
        IntakeLeft.setPower(ExtendPower);
        IntakeRight.setPower(ExtendPower);

        telemetry.addData("Lift pos ", LiftPos);
        telemetry.addData("Extend pos ", ExtendPos);
        telemetry.addData("lift target ", LiftTarget);
        telemetry.addData("extend target ", ExtendTarget);
        telemetry.update();

    }

}

