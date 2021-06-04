package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;

import static org.firstinspires.ftc.teamcode.AutoFunctions.*;

@Autonomous(name = "Auto", group = "OpModes")
public class Auto extends UpliftAuto {

    UpliftRobot robot = new UpliftRobot(this);

    @Override
    public void initHardware() {

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        driveToPosition(0, 50, 0.7, 2, robot);
        driveToPosition(0, 0, 0.7, 2, robot);
    }

    @Override
    public void exit() throws InterruptedException {

    }
}
