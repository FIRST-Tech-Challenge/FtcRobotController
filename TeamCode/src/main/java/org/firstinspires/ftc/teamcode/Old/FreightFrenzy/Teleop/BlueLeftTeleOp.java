package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Teleop;

import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis.ypos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.BasicChassis;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot;

@Config
@TeleOp(name = "BlueLeftTeleopRegionals")
@Disabled

public class BlueLeftTeleOp extends LinearOpMode {
    public void runOpMode() {
        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ENCODER, true ,true,0);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        double[] xcoords = new double[4];
        double[] ycoords = new double[4];
        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();
        double startx = 0;
        double starty = 0;


        //Aiden - during competition day robot disconnected so we are trying this code
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            double x = (double)xpos - 8.75;
            double y = (double)ypos - 6.25;

            packet.fieldOverlay().setFill("blue").fillRect(x, y, 15, 15);
            telemetry.addData("status", "waiting for start command...");
            telemetry.addData("xpos",x);
            telemetry.addData("ypos",y);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            //12.5 and 17.5
            /*
            xcoords[0] = (double)xpos-6.25;
            xcoords[1] = (double)xpos+6.25;
            xcoords[2] = (double)xpos-6.25;
            xcoords[3] = (double)xpos+6.25;

            ycoords[0] = (double)ypos-8.75;
            ycoords[1] = (double)ypos+8.75;
            ycoords[2] = (double)ypos-8.75;
            ycoords[3] = (double)ypos+8.75;
            */
            robot.teleopLoop(1,35.5,0);
        }


        idle();
    }
}