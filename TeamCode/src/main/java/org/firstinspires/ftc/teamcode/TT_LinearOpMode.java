
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

@TeleOp(name="Linear OpMode", group="Techtonics")
public class TT_LinearOpMode extends LinearOpMode {
    private TT_RobotHardware robot = new TT_RobotHardware(this);

    private double openPos = 0.03;
    private double closedPos = 0.07;

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.driveRobot();
            robot.drivelift();
            robot.moveLiftArm();
            robot.displayTelemetry();
        }
    }
}
