package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.PivotBot;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "PivotSlideTest")
public class PivotSlideTest extends LinearOpMode {
//    private PivotBot robot = new PivotBot(this);

    private PivotBot robot = new PivotBot(this);
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = false;

        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
         Timer LoopTime = new Timer();
            telemetry.setMsTransmissionInterval(11);

            robot.onLoop(0, "manual drive");


            robot.slideControl(gamepad1.dpad_right, gamepad1.dpad_left);

//            robot.climbControl(gamepad1.a);
            double lTime = LoopTime.getElapsedTimeSeconds();
            telemetry.addData("slide position", robot.getSlidePosition());
            telemetry.addData("looptime", lTime);
            telemetry.addData("slide target pos", robot.slideTarget);
            telemetry.addData("Slide1motorpower",robot.getSlideMotor1Power());
            telemetry.addData("slide2motorpower", robot.getSlideMotor2Power());

            telemetry.addData("robot angle", Math.toRadians(robot.getPivotPosition()*360/8192));
            if (robot.pivotOutOfRange) {

                telemetry.addData("ERROR", "Pivot is out of Range");

            }
            telemetry.update();


        }
        robot.close();
    }

}
