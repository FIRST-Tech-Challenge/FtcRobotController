package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.PIDControlAutoOpMode;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.sample.SamplePIDAutoOpMode;

/**
 * Blue corner PID controlled Robotic centric auto up mode
 * <p>Commented out code shows how to switch to field centric driver</p>
 * <p>see @{link {@link SamplePIDAutoOpMode#setupWaypointsAndActions()}} for how to setup trajectory and actions</p>
 */
@Autonomous(name = "blue PID r-centric auto mode", preselectTeleOp = "main_teleop")
public class BluePIDRobotCentricAutoOpMode extends PIDControlAutoOpMode {
    @Override
    protected void setupWaypointsAndActions() {

    }

//    @Override
//    protected AutoRobotDriver createRobotDriver() {
//        return new FieldCentricDriver(driveTrain);
//    }
}
