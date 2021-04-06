package org.firstinspires.ftc.teamcode.opmodes.hardwaretests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.DROP_TRANSFER;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.FLICKER_POS;
import static org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants.RAISE_TRANSFER;

@Autonomous(name = "Transfer Tester", group = "Hardware Tester")
public class TransferTester extends UpliftAuto {
    UpliftRobot robot;
    TransferSubsystem transfer;
    FtcDashboard ftcDashboard;
    Telemetry dashTelem;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        transfer = robot.transferSub;
    }

    @Override
    public void initAction() {
        ftcDashboard = FtcDashboard.getInstance();
        dashTelem = ftcDashboard.getTelemetry();
        transfer.initTransferPos();
    }

    @Override
    public void body() throws InterruptedException {
        waitForStart();

        while(opModeIsActive()) {
            if(RAISE_TRANSFER) {
                transfer.teleRaiseTransfer();
                RAISE_TRANSFER = false;
            } else if(DROP_TRANSFER) {
                transfer.teleDropTransfer();
                DROP_TRANSFER = false;
            }
        }

    }

    @Override
    public void exit() {
        robot.stopThreads();
    }

}
