package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Color Sensor Value Test",group="chrisBot")

public class colorSensorValueTest extends LinearOpMode {
    chrisBot robot = new chrisBot();
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        telemetry.setAutoClear(true);
        while(!isStopRequested()) {
            telemetry.addData("Left sensor Alpha",robot.colorL.alpha());
            telemetry.addData("Left sensor Red",robot.colorL.red());
            telemetry.addData("Left sensor Green",robot.colorL.green());
            telemetry.addData("Left sensor Blue",robot.colorL.blue());
            telemetry.addData("Right sensor Alpha",robot.colorR.alpha());
            telemetry.addData("Right sensor Red",robot.colorR.red());
            telemetry.addData("Right sensor Green",robot.colorR.green());
            telemetry.addData("Right sensor Blue",robot.colorR.blue());
            telemetry.update();
        }
    }
}
