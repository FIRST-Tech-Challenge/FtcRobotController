package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;

@Autonomous(name = "\uD83D\uDD35BlueNearBoardCenter", group = "Auto")
public class BlueNearBoardCenter extends NearBoard {
    public BlueNearBoardCenter()
    {
        super(Alliance.BLUE,ParkPosition.CENTER);
    }
}