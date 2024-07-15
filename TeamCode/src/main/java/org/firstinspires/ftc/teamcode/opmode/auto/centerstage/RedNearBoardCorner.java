package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;

@Autonomous(name = "\uD83D\uDD25RedNearBoardCorner", group = "Auto")
public class RedNearBoardCorner extends NearBoard {
    public RedNearBoardCorner()
    {
        super(Alliance.RED,ParkPosition.CORNER);
    }
}