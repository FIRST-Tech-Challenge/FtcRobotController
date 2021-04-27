package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

public class TransferCommands extends Command {

    public TransferSubsystem transfer;
    public UpliftTele opMode;
    public boolean pressed;
    UpliftRobot robot;

    public TransferCommands(UpliftTele opMode, UpliftRobot robot, TransferSubsystem transferSubsystem) {
        super(opMode, transferSubsystem);
        this.transfer = transferSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {
        transfer.initTransferPos();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        // if transfer going down and bottom touch sensor is pressed, stop
        if (!transfer.transferTouchBottom.getState() && opMode.gamepad2.right_stick_y > 0) {
            transfer.setTransferPower(0);
        // else, set the transfer power to 1/2 of the joystick val
        } else {
            transfer.setTransferPower(Range.clip(opMode.gamepad2.right_stick_y/1.5, -1, 1));
        }

        // if Y pressed on OPERATOR gamepad, raise transfer
        // *NOTE: Y button on the OPERATOR gamepad also turns on shooter to the highgoal velocity (simultaneously)
        if (opMode.gamepad2.y) {
//            if(robot.highGoalMode) {
                robot.setShootingState(UpliftRobot.ShootingState.PREPARING_HIGHGOAL);
                transfer.teleRaiseTransfer();
                robot.setShootingState(UpliftRobot.ShootingState.SHOOTING_HIGHGOAL);
            }
//        else {
        if(opMode.gamepad2.x || robot.shootingState == UpliftRobot.ShootingState.PREPARING_POWERSHOT) {
            robot.setShootingState(UpliftRobot.ShootingState.PREPARING_POWERSHOT);
            if(robot.transferState != UpliftRobot.TransferState.UP) {
                transfer.teleRaiseTransfer();
            }
        }
//            }
//        }

        // if done shooting, drop the transfer and set the shooter state to IDLE
        // *NOTE: DONE_SHOOTING also tells the shooter to set power to 0.1
        if (robot.shootingState == UpliftRobot.ShootingState.DONE_SHOOTING) {
            transfer.teleDropTransfer();
            robot.setShootingState(UpliftRobot.ShootingState.IDLE);
        }
    }

    @Override
    public void stop() {
        transfer.setTransferPower(0);
    }
}
