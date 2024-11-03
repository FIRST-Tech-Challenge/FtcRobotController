package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.SimpleAutoOpMode;

/**
 * Blue corner simple kP controlled Robotic centric auto up mode
 * <p>Commented out code shows how to switch to field centric driver</p>
 */
@Autonomous(name = "blue simple r-centric auto mode", preselectTeleOp = "main_teleop")
public class BlueSimpleRobotCentricAutoOpMode extends SimpleAutoOpMode {
    @Override
    protected void setupWaypointsAndActions() {

    }

//    @Override
//    protected AutoRobotDriver createRobotDriver() {
//        return new FieldCentricDriver(driveTrain);
//    }
}
