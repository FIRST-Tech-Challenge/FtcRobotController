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

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        // if transfer going down and touch is pressed, stop
        if (!transfer.transferTouchBottom.getState() && opMode.gamepad2.right_stick_y > 0) {
            transfer.setTransferPower(0);
        } else if (!transfer.transferTouchTop.getState() && opMode.gamepad2.right_stick_y < 0) {
            transfer.setTransferPower(0);
        } else {
            transfer.setTransferPower(Range.clip(-opMode.gamepad2.right_stick_y / 2, -1, 1));
        }


        if (opMode.gamepad2.y) {
            robot.setShootingState(UpliftRobot.ShootingState.PREPARING_TO_SHOOT);
            transfer.raiseTransfer();
            robot.setShootingState(UpliftRobot.ShootingState.SHOOTING);
        }

        if (opMode.gamepad2.x) {
            robot.setShootingState(UpliftRobot.ShootingState.PREPARING_TO_SHOOT);
            transfer.raiseTransfer();
            robot.setShootingState(UpliftRobot.ShootingState.SHOOTING);
        }


        if (robot.shootingState == UpliftRobot.ShootingState.DONE_SHOOTING) {
            transfer.dropTransfer();
            robot.setShootingState(UpliftRobot.ShootingState.IDLE);
        }
    }

    @Override
    public void stop() {

    }
}
