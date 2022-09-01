package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.ypos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.RFModules.Attachments.RFAngleAdjust;
import org.firstinspires.ftc.teamcode.Components.RFModules.Attachments.RFTurret;
import org.firstinspires.ftc.teamcode.BlackoutRobot;

import java.util.ArrayList;

@Config
@TeleOp(name = "RFModuleTeleop")
//@Disabled

public class RFModuleTeleOp extends LinearOpMode {

    public void runOpMode() {
        ArrayList<Double> coefs = new ArrayList<>();
        coefs.add(4.0);
        coefs.add(400.0);
        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ENCODER, true ,true
                ,0);
        RFAngleAdjust angleAdjust = new RFAngleAdjust(this, "turret_Angle_Control",
                "turret_Angle_Control2", 118.0/270);
        RFTurret turretRotation = new RFTurret("turret_Rotation", DcMotor.RunMode.RUN_USING_ENCODER,
                true, coefs, 570, -570);

        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();


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


            if (gamepad2.y) {
                angleAdjust.flipServos();
            }
            if (gamepad2.left_stick_x != 0 && gamepad2.a) {
                turretRotation.setPosition(gamepad2.left_stick_x);
            }
        }


        idle();
    }

}