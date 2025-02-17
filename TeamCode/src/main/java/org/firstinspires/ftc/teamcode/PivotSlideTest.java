package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.GyroBot;
import org.firstinspires.ftc.teamcode.bots.HangBot;
import org.firstinspires.ftc.teamcode.bots.AutomationBot;
import org.firstinspires.ftc.teamcode.bots.LimelightBot;
import org.firstinspires.ftc.teamcode.bots.PivotBotTest;
import org.firstinspires.ftc.teamcode.bots.PinchBot;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.sample.Sample;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.teamcode.bots.PivotBot;

@TeleOp(name = "PivotSlideTest")
public class PivotSlideTest extends LinearOpMode {
//    private PivotBot robot = new PivotBot(this);

    private PivotBotTest robot = new PivotBotTest(this);
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

            robot.pivotControlTest(gamepad1.dpad_up, gamepad1.dpad_down);
            robot.slideControl(gamepad1.dpad_right, gamepad1.dpad_left);

//            robot.climbControl(gamepad1.a);
            double lTime = LoopTime.getElapsedTimeSeconds();
            telemetry.addData("slide position", robot.getSlidePosition());
            telemetry.addData("pivot position", robot.getPivotPosition());
            telemetry.addData("looptime", lTime);
            telemetry.addData("pivot targetposition",robot.pivotTarget);
            telemetry.addData("slide target pos", robot.slideTarget);
            dashboardTelemetry.addData("slide position", robot.getSlidePosition());
            dashboardTelemetry.addData("pivot position", robot.getPivotPosition());
            dashboardTelemetry.addData("looptime", lTime);
            dashboardTelemetry.addData("targetposition",robot.pivotTarget);
            dashboardTelemetry.addData("robot angle", Math.toRadians(robot.getPivotPosition()*360/8192));
            telemetry.addData("robot angle", Math.toRadians(robot.getPivotPosition()*360/8192));
            if (robot.pivotOutOfRange) {

                telemetry.addData("ERROR", "Pivot is out of Range");

            }
            telemetry.update();


        }
        robot.close();
    }

}
