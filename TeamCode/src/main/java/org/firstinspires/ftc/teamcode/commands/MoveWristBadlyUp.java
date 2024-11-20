package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.WristSub;
/**

 * This command is dedicated to a command that controls the wrist for the Tele-op mode
 */

public class MoveWristBadlyUp extends CommandBase {

    private final WristSub wristSub;
    private final Telemetry telemetry;
    /**
     * This command deals with the wrist in teleop.
     *
     * @param wristSubParam The wrist sub to be imported
     */

    public MoveWristBadlyUp(WristSub wristSubParam, Telemetry telemetry) {
        this.wristSub = wristSubParam;
        this.telemetry = telemetry;
        addRequirements(this.wristSub);
    }

    @Override
    public void execute() {
        wristSub.setPosition(wristSub.getPosition() + 0.05);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}