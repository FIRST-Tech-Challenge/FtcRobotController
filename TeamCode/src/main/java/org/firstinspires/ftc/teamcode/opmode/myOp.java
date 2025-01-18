package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

@TeleOp
public class myOp extends CommandOpMode {
    protected RobotContainer m_robot;

    @Override
    public void initialize() {
        m_robot= new RobotContainer(hardwareMap, new BTController(gamepad1));
        enable();

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            m_robot.period();
            run();
        }
        reset();
    }
}