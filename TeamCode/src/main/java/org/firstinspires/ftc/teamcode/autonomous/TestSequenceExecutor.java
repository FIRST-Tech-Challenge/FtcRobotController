package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.autonomous.sequences.MovementActionSequence;
import org.firstinspires.ftc.teamcode.autonomous.sequences.TestActionSequence;
import org.firstinspires.ftc.teamcode.hardware.MovementHardware;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Autonomous;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "test sequence")
public class TestSequenceExecutor extends UltimateGoalHardware implements Autonomous {

    @Override
    public void init() {
        super.init();
        this.initializeForAutonomous(this);
    }

    @Override
    public void run_loop() {

    }

    @Override
    public Localizer.RobotTransform getStartingTransform() {
        Position position = new Position(DistanceUnit.INCH, -63, -48, 0, System.nanoTime());
        double heading = 90;
        Localizer.RobotTransform transform = new Localizer.RobotTransform(position, heading);
        return transform;
    }

    @Override
    public ActionSequence getActionSequence() {
        return new TestActionSequence();
    }
}
