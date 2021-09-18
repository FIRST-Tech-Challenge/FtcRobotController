package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotData.DeviceMap;

@Autonomous(name = "Auto")
public class AutonomousOpMode extends LinearOpMode {
    DeviceMap hw = new DeviceMap();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){

    }
}
