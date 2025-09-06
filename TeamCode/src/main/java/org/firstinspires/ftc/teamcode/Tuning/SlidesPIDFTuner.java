package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

// gotta rename this but otherwise
// ----- READY TO TRANSFER ----- //

@Config
@TeleOp(name = "Vertical slides PIDF Tuner", group = "Tuning")
public class SlidesPIDFTuner extends LinearOpMode {

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    Robot robot;

    public static double kp = 0.02;
    public static double ki = 0.3;
    public static double kd = 0.001;
    public static double kf = 0;

    double currentPosition;
    boolean TorF = false;

    public static double goal = 0;

    double prevGoal;

    public ElapsedTime startTime= new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        PIDController slideController = new PIDController(kp, ki, kd);

        waitForStart();

        while(opModeIsActive()){

            if(prevGoal != goal) {
                startTime.reset();
                prevGoal = goal;
            }

            slideController.setPID(kp, ki, kd);
            if(TorF) {currentPosition = robot.getSlides().getSlidePositionAvg();}
            else{currentPosition = robot.getSlides().getOneSlidePosition();}

            robot.getSlides().setSlidePower(slideController.calculate(currentPosition, goal) + kf);

            t.addData("Current Position", currentPosition);
            t.addData("Goal Position", goal);
            t.update();

        }
    }
}
