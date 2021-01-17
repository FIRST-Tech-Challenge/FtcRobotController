package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.autonomous.sequences.MovementActionSequence;
import org.firstinspires.ftc.teamcode.hardware.MovementHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Autonomous;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "movement")
public class MovementSequenceExecutor extends MovementHardware implements Autonomous {

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
        return null;
    }

    @Override
    public ActionSequence getActionSequence() {
        return new MovementActionSequence();
    }
}