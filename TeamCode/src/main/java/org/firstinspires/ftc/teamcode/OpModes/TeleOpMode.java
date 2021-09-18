package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotData.DeviceMap;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends OpMode {
    DeviceMap hw = new DeviceMap();

    //Runs once when "Init" button is pressed
    @Override
    public void init() {
        hw.init(hardwareMap);
    }

    //Runs repeatedly after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Runs once when "Start" button is pressed
    @Override
    public void start() {
    }

    //Runs repeatedly after the driver hits START, but before they hit STOP
    @Override
    public void loop() {
    }

    //Runs once when "Stop" button is pressed
    @Override
    public void stop() {
    }
}
