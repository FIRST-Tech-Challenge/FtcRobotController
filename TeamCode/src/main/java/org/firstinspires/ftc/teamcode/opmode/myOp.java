package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
@TeleOp
public class myOp extends CommandOpMode {
    RobotContainer m_robot;
    double period=0.03;

    @Override
    public void initialize() {
        m_robot= new RobotContainer(hardwareMap,gamepad1,gamepad2);
        enable();

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {

            run();
        }
        reset();
    }
}