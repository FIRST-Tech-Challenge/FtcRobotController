package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;

@Autonomous(name = "\uD83D\uDD35BlueNearBoardCorner", group = "Auto")
public class BlueNearBoardCorner extends NearBoard {
    public BlueNearBoardCorner()
    {
        super(Alliance.BLUE,ParkPosition.CORNER);
    }
}