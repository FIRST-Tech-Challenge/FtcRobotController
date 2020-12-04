package org.firstinspires.ftc.teamcode.examples.clawbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.structure.CommandOpMode;

import org.firstinspires.ftc.teamcode.examples.clawbot.OperatorInterface;
import org.firstinspires.ftc.teamcode.examples.clawbot.Robot;

@TeleOp(name = "Claw TeleOo")
public class ClawBotTeleOp extends CommandOpMode {
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
