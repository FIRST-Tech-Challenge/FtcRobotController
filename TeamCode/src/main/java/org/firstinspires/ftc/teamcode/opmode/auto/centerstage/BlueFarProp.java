package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;

@Autonomous(name = "\uD83D\uDD35BlueFarPropPark", group = "Auto")
public class BlueFarProp extends FarNoBoard {
    public BlueFarProp()
    {
        super(Alliance.BLUE);
    }
}