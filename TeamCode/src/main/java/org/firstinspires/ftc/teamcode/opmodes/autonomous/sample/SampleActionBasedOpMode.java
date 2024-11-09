package org.firstinspires.ftc.teamcode.opmodes.autonomous.sample;

import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.ActionBasedOpMode;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.BaseMovement;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.ForwardMovement;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.RotationMovement;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.StrafeMovement;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.LinkedList;
import java.util.Queue;

import kotlin.Unit;

@Autonomous(name = "sample action based", preselectTeleOp = "main_teleop")
public class SampleActionBasedOpMode extends ActionBasedOpMode {

    @Override
    protected void setupWaypointsAndActions() {
        int sideLen = 30;
        Queue<BaseMovement> movements = new LinkedList<>();
//        movements.add(new ForwardMovement(Units.inchesToMeters(sideLen))); //rotation takes radians, movement takes meters
//        movements.add(new RotationMovement(Units.degreesToRadians(90)));
//        movements.add(new ForwardMovement(Units.inchesToMeters(sideLen)));
//        movements.add(new RotationMovement(Units.degreesToRadians(90)));
//        movements.add(new ForwardMovement(Units.inchesToMeters(sideLen)));
//        movements.add(new RotationMovement(Units.degreesToRadians(90)));
//        movements.add(new ForwardMovement(Units.inchesToMeters(sideLen)));
//        movements.add(new RotationMovement(Units.degreesToRadians(90)));

//        movements.add(new ForwardMovement(Units.inchesToMeters(20)));
//        movements.add(new RotationMovement(Units.degreesToRadians(90)));
//        movements.add(new ForwardMovement(Units.inchesToMeters(20)));
//        movements.add(new RotationMovement(Units.degreesToRadians(90)));
//        movements.add(new ForwardMovement(Units.inchesToMeters(20)));
//        movements.add(new RotationMovement(Units.degreesToRadians(90)));
//        movements.add(new ForwardMovement(Units.inchesToMeters(20)));
//        movements.add(new RotationMovement(Units.degreesToRadians(90)));

        movements.add(new RotationMovement(Units.degreesToRadians(45)));
        movements.add(new ForwardMovement(Units.inchesToMeters(34)));
        movements.add(new RotationMovement(Units.degreesToRadians(-45)));
        movements.add(new RotationMovement(Units.degreesToRadians(170)));
        movements.add(new ForwardMovement(Units.inchesToMeters(20)));
        movementQueue.add(movements);

        actions.add(new RunCommand(() -> {

        }));
    }
}
