package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class DashboardGraphTest extends OpMode {
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry telem = dashboard.getTelemetry();
    public ElapsedTime timer = new ElapsedTime();
    public Telemetry telemetry2 = new MultipleTelemetry(telemetry, telem);
    public static int num = 1;
    @Override
    public void init() {
        telemetry2.addData("Number: ", num);
        telemetry2.update();
    }

    @Override
    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {
        if(timer.seconds() > 3){
            num = num + 1;
            telemetry2.addData("Number: ", num);
            telemetry2.update();
            timer.reset();
        }
    }
}
