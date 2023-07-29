package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Tests;

import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis.ypos;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.BasicChassis;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFAngleAdjust;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFBasket;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFFlippingIntake;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFIntake;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFSlides;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFTurret;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFCRServo;

import java.util.ArrayList;
@Disabled
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

        RFAngleAdjust angleAdjust = new RFAngleAdjust("angleAdjust", "turret_Angle_Control",
                "turret_Angle_Control2", 118.0/270);
        RFTurret turretRotation = new RFTurret("turret_Rotation", DcMotor.RunMode.RUN_USING_ENCODER,
                true, rotationCoefs, 570, -570);
        RFSlides turretExtension = new RFSlides("turret_Extension", DcMotor.RunMode.RUN_USING_ENCODER,
                true, extensionCoefs, 1000, 0);

        RFIntake intakeMotor = new RFIntake("IntakeMotor", DcMotor.RunMode.RUN_USING_ENCODER, true);
        RFFlippingIntake intakeServos = new RFFlippingIntake("flippingIntake", "IntakeServo", "IntakeServo2", 1);
        RFBasket basketServo = new RFBasket("basketActuationServo", 0.75);
        RFBasket basketArmServo = new RFBasket("basketArmServo", 0.75);
        RFCRServo carouselCRServo = new RFCRServo("carousel");

        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();

        logger.log("/RobotLogs/GeneralRobot", "Running: RFModuleTeleOp\n");

        //Aiden - during competition day robot disconnected so we are trying this code
        while (opModeIsActive() && !isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();
            double x = xpos - 8.75;
            double y = ypos - 6.25;

            logger.loopCounter++;

            packet.fieldOverlay().setFill("blue").fillRect(x, y, 15, 15);
            telemetry.addData("status", "waiting for start command...");
            telemetry.addData("xpos",x);
            telemetry.addData("ypos",y);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();


            if (gamepad2.y) {
                angleAdjust.flipServosMax();
            }
            if (gamepad2.a) {
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
                intakeServos.flipServosInterval(0.21, 1);
            }
            if (gamepad1.a) {
                basketServo.flipServoInterval(0.2, 0.75);
            }
            if (gamepad1.x) {
                basketArmServo.flipServoMax();
            }

            if (gamepad1.dpad_right) {
                carouselCRServo.spinClockwise();
            }

            if (gamepad1.dpad_left) {
                carouselCRServo.spinCounterClockwise();
            }

            if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
                carouselCRServo.stopSpinning();
            }
        }

        logger.log("/RobotLogs/GeneralRobot", "Program stopped normally. ");

        idle();
    }

}