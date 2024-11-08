package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.BaseMovement;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.ActionBasedOpMode;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.ForwardMovement;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.RotationMovement;
import org.firstinspires.ftc.teamcode.util.Units;


import java.util.Queue;
import java.util.LinkedList;

//@Autonomous(name = "Test Auto 1", preselectTeleOp = "main_teleop")
public class TestAuto1 extends ActionBasedOpMode {

    @Override
    protected void setupWaypointsAndActions() {
        Queue<BaseMovement> movements = new LinkedList<>();
        movements.add(new ForwardMovement(Units.inchesToMeters(23d)));
        movements.add(new RotationMovement(-90));
        movements.add(new ForwardMovement(Units.inchesToMeters(79d)));
        movements.add(new RotationMovement(90));
        movements.add(new ForwardMovement(Units.inchesToMeters(23d)));
        movementQueue.add(movements);
    }
}
