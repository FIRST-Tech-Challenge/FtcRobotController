package org.firstinspires.ftc.teamcode.opmode.autos;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class periodOpMode extends CommandOpMode {
    RobotContainer m_robot;
    private final int periodMs = 40;//Seconds
    private final int periodUs=periodMs*1000;
    private long nextPeriodTimeUs = 0;

    @Override
    public void initialize() {
        m_robot = new RobotContainer(hardwareMap, new BTController(gamepad1));
        enable();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            long nowUs=System.nanoTime() / 1000;
            if (nextPeriodTimeUs < nowUs) {
                nextPeriodTimeUs=nowUs;
            }else {
                long sleepTimeMs = (nextPeriodTimeUs - nowUs) / 1000;
                if (sleepTimeMs >  1) {
                    sleep(sleepTimeMs -  1);
                }
                while (nextPeriodTimeUs > System.nanoTime() / 1000){}
            }
            nextPeriodTimeUs+=periodUs;
            period();//used for recording controllers
            m_robot.period();//used for telemetry
            run();//runs all commands and subsystems' periodic
        }
        endFunction();
        reset();
    }

    protected void period(){}
    protected void endFunction(){}
}
