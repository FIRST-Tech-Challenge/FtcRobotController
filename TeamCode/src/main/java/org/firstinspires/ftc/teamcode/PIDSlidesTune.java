package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "PID tune")
public class PIDSlidesTune extends LinearOpMode{
    FtcDashboard dashboard;

    public  static double targetposition = 2000;
    public  static double P = 0.046;
    public  static double I = 0.000002;
    public  static double D = 0.0855;
    public DcMotor slides;
    @Override
    public void runOpMode() throws InterruptedException {
        Slide slide = new Slide();
        slides = hardwareMap.dcMotor.get("slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime timer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!"); //hehe
        telemetry.addData("Slides", slides.getCurrentPosition());
        telemetry.update();
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            if(opModeIsActive()) {

                slide.slidego(targetposition, hardwareMap);

                telemetry.addData("Slides", slides.getCurrentPosition());
                telemetry.update();
            }
        }
        }
    }