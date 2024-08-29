package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;

@Autonomous
public abstract class Auto extends AutoControl {
    @Override
    public void start(){
        super.start();
        AutoDrive(10, 0);
        AutoTurn(0);
        AutoDrive(10, 90);
        AutoTurn(0);
        AutoDrive(10, 180);
        AutoTurn(0);
        AutoDrive(10, 270);
    }
}
