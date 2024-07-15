package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;

@Autonomous(name = "\uD83D\uDD25RedNearBoardCenter", group = "Auto")
public class RedNearBoardCenter extends NearBoard {
    public RedNearBoardCenter()
    {
        super(Alliance.RED,ParkPosition.CENTER);
    }
}