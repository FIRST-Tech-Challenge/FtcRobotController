package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class OutputTest extends LinearOpMode {

    Output output;

    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());



        output = new Output(hardwareMap);
        waitForStart();
        output.setComponentPositionsFromOutputEndPoint(
                new OutputEndPoint(new Point2d(-17.5, 43.3), 206, 0, false)
        );
        while (opModeIsActive()) {
            if (gamepad1.a) {
                output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(new Point2d(2, 30), 0, 0, false));
            } else if (gamepad1.b) {
                output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(new Point2d(-4, 34), 90, 0, false));
            } else if (gamepad1.x) {
                output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(new Point2d(5.7, 15.8), 90, 0, false));
            } else if (gamepad1.y) {
                output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(new Point2d(-17.7, 16.8), 90, 0, false));
            } else {
                output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(new Point2d(9.5, 24.2), 0, 0, false));
            }
            output.sendVerticalSlidesToTarget();


            telemetry.addLine(output.currentPosition().pointTelemetry());
            telemetry.addLine(output.currentPosition().componentValuesIrl());
            telemetry.update();
        }
    }
}
