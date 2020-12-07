package org.firstinspires.ftc.teamcode.examples.clawbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.teamcode.examples.clawbot.OperatorInterface;
import org.firstinspires.ftc.teamcode.examples.clawbot.Robot;
import org.firstinspires.ftc.teamcode.examples.clawbot.subsystems.ClawSubsystem;

@TeleOp(name = "Claw TeleOo")
public class ClawBotTeleOp extends CommandOpMode implements Loggable {
    public Robot robot;
    public OperatorInterface OI;

    @Override
    public void uponInit() {
        robot = new Robot();
        OI = new OperatorInterface(driverGamepad, robot);
    }

    @Override
    public void runLoop() {
        robot.drivebaseSubsystem.arcadeDrive(driverGamepad.leftStickY.getAsDouble(), driverGamepad.leftStickX.getAsDouble());
    }

}
