package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Old.Components.Chassis.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Old.Components.Chassis.EncoderChassis.ypos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.Components.Chassis.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.RFModules.Attachments.RFAngleAdjust;
import org.firstinspires.ftc.teamcode.Components.RFModules.Attachments.RFFlippingIntake;
import org.firstinspires.ftc.teamcode.Components.RFModules.Attachments.RFIntake;
import org.firstinspires.ftc.teamcode.Components.RFModules.Attachments.RFSlides;
import org.firstinspires.ftc.teamcode.Components.RFModules.Attachments.RFTurret;
import org.firstinspires.ftc.teamcode.Old.Robots.BlackoutRobot;

import java.util.ArrayList;

@Config
@TeleOp(name = "RFModuleTeleop")
//@Disabled

public class RFModuleTeleOp extends LinearOpMode {

    public void runOpMode() {
        boolean intakeup = false;
        ArrayList<Double> rotationCoefs = new ArrayList<>();
        rotationCoefs.add(4.0);
        rotationCoefs.add(400.0);

        ArrayList<Double> extensionCoefs = new ArrayList<>();
        extensionCoefs.add(4.0);
        extensionCoefs.add(400.0);

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();


        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ENCODER, true ,true
                ,0);

        RFAngleAdjust angleAdjust = new RFAngleAdjust("turret_Angle_Control",
                "turret_Angle_Control2", 118.0/270);
        RFTurret turretRotation = new RFTurret("turret_Rotation", DcMotor.RunMode.RUN_USING_ENCODER,
                true, rotationCoefs, 570, -570);
        RFSlides turretExtension = new RFSlides("turret_Extension", DcMotor.RunMode.RUN_USING_ENCODER,
                true, extensionCoefs, 1000, 0);

        RFIntake intakeMotor = new RFIntake("IntakeMotor", DcMotor.RunMode.RUN_USING_ENCODER, true);
        RFFlippingIntake intakeServos = new RFFlippingIntake("IntakeServo", "IntakeServo2", 1);

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
                turretRotation.setPosition(gamepad2.left_stick_x * 570);
            }
            if (gamepad2.right_trigger != 0 && gamepad2.right_bumper) {
                turretExtension.setPosition(gamepad2.right_trigger * 1000);
            }
            if (gamepad2.left_trigger != 0 && gamepad2.left_bumper) {
                turretExtension.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                intakeMotor.setVelocity(1500);
                intakeMotor.getVelocity();
            }
            if (gamepad1.b) {
                intakeMotor.setVelocity(0);
                intakeMotor.getVelocity();
            }
            if (gamepad1.left_bumper) {
                if (intakeup) {
                    intakeServos.setPositions(0.21);
                    intakeup = false;
                    //stop looking here willy
                }
                else {
                    intakeServos.setPositions(1);
                    intakeup = true;
                }
            }
        }


        idle();
    }

}