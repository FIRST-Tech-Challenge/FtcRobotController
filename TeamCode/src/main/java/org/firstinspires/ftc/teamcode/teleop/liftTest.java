package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.config.Hardware2;
public class liftTest extends LinearOpMode {
    Hardware2 robot = new Hardware2();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleOpIMU();
        while (opModeIsActive())
        {
            robot.verticalLiftMotor.setPower(0.4);
        }
    }
}
