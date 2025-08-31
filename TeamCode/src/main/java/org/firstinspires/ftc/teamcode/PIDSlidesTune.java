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

    public  static double targetposition = 8000;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slides = hardwareMap.dcMotor.get("slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Slides", slides.getCurrentPosition());
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime timer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!"); //hehe
        telemetry.addData("Slides", slides.getCurrentPosition());
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            if(opModeIsActive()) {
                Slide slide = new Slide();
                slide.slidesinit(hardwareMap);
                slide.slidego(targetposition);
            }
        }
        }
    }