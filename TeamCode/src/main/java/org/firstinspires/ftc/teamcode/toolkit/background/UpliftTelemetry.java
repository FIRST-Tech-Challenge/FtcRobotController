package org.firstinspires.ftc.teamcode.toolkit.background;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Background;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

public class UpliftTelemetry extends Background {

    UpliftRobot robot;
    LinearOpMode opMode;
    Telemetry telem;

    public UpliftTelemetry(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.opMode = robot.opMode;
//        this.telem = FtcDashboard.getInstance().getTelemetry();
        this.telem = robot.opMode.telemetry; //
    }

    @Override
    public void loop() {

        if(opMode instanceof UpliftTele) {
//            telem.addData("time elapsed", opMode.getRuntime());
//            telem.addData("X Position", robot.worldX);
//            telem.addData("shooter1 power", robot.shooter1.getPower());
//            telem.addData("shooter2 power", robot.shooter2.getPower());

//            telem.update();
        } else if(opMode instanceof UpliftAuto){

        }

    }

}
