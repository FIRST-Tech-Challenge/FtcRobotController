package org.firstinspires.ftc.teamcode.Configs.newConfig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class senseColor extends BaseNewOpMode {
    HardwareNew robot = new HardwareNew();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            //telemetry.addData("Red ", String.valueOf(robot.getColorSensor().red()/100));
            //telemetry.addData("Blue: ", String.valueOf(robot.getColorSensor().blue()/100));
            //telemetry.addData("Green: ", String.valueOf(robot.getColorSensor().green()/100));
            //telemetry.update();
        }
    }
}
