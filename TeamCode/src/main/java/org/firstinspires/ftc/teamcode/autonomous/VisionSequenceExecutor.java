package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.autonomous.sequences.TestActionSequence;
import org.firstinspires.ftc.teamcode.autonomous.sequences.VisionActionSequence;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.hardware.VisionHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Autonomous;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "vision sequence")
public class VisionSequenceExecutor extends VisionHardware implements Autonomous {

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
        return new VisionActionSequence();
    }
}
