package org.firstinspires.ftc.teamcode.commandBased;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;
import org.firstinspires.ftc.teamcode.rr.util.DashboardUtil;

import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

public class BaseOpMode extends CommandOpMode {

    protected DrivetrainSubsystem drivetrainSS;
    protected ElevatorSubsystem elevatorSS;
    protected ArmSubsystem armSS;
    protected RotatorSubsystem rotatorSS;
    protected IntakeSubsystem intakeSS;

    protected ReforgedGamepad driver;
    protected ReforgedGamepad operator;

    protected TelemetryPacket packet;
    protected Canvas fieldOverlay;

    protected MultipleTelemetry tele;

    @Override
    public void initialize() {
        drivetrainSS = new DrivetrainSubsystem(hardwareMap);
        elevatorSS = new ElevatorSubsystem(hardwareMap);
        armSS = new ArmSubsystem(hardwareMap);
        rotatorSS = new RotatorSubsystem(hardwareMap);
        intakeSS = new IntakeSubsystem(hardwareMap);

        driver = new ReforgedGamepad(gamepad1);
        operator = new ReforgedGamepad(gamepad2);

        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();

        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if (Constants.DISPLAY) {
            // Draw the target on the field
            fieldOverlay.setStroke("#dd2c00");
            fieldOverlay.strokeCircle(Constants.TARGET.getX(), Constants.TARGET.getY(), 3);

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, drivetrainSS.getPose());

            //send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }

        if (Constants.DEBUG_DRIVE) {

        }

        if (Constants.DEBUG_ELE) {
            tad("ele pos", elevatorSS.getElePos());
            tad("ele profile target", elevatorSS.getEleProfileTarget());
            tad("ele final target", elevatorSS.getEleTarget());
            tad("ele power", elevatorSS.getElePower());
        }

        if (Constants.DEBUG_ARM) {
            tad("arm final encoder target", armSS.getArmTargetEnc());
            tad("arm final angle target", armSS.getArmTargetAngle());
            tad("arm profile target", armSS.getArmProfileTarget());
            tad("arm target", armSS.getArmTargetEnc());
            tad("arm pos", armSS.getArmPos());
            tad("arm power", armSS.getArmPower());
            tad("arm angle", armSS.getArmAngle());
            tad("arm velocity", armSS.getArmVelocity());
            tad("arm acceleration", armSS.getArmAcceleration());
            tad("arm KF", armSS.getCoeffs()[6]);
        }

        if (Constants.DEBUG_ROTATOR) {
            tad("rotator pos", rotatorSS.getPosition());
            tad("rotator usFrame", rotatorSS.getPWMRange()[0]);
            tad("rotator usPulseLower", rotatorSS.getPWMRange()[1]);
            tad("rotator usPulseUpper", rotatorSS.getPWMRange()[2]);
            tad("rotator current", rotatorSS.getAverageCurrent());
        }

        if (Constants.DEBUG_INTAKE) {
            tad("intake power", intakeSS.getPower());
            tad("intake current", intakeSS.getServoBusCurrent());
            tad("intake avg current", intakeSS.getAverageCurrent());
        }
    }

    protected void tad(String caption, Object value) {
        tele.addData(caption, value);
    }
}
