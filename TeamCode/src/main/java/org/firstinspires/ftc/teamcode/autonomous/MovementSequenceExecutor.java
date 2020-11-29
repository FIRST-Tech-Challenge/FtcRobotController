package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.autonomous.sequences.MovementActionSequence;
import org.firstinspires.ftc.teamcode.hardware.MovementHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Autonomous;
import org.firstinspires.ftc.teamcode.playmaker.AutonomousExecutor;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

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
    public ActionSequence getActionSequence() {
        return new MovementActionSequence();
    }
}
