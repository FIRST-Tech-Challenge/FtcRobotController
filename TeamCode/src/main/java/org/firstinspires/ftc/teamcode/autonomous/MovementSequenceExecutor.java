package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.autonomous.sequences.MovementActionSequence;
import org.firstinspires.ftc.teamcode.hardware.MovementHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Autonomous;
import org.firstinspires.ftc.teamcode.playmaker.AutonomousExecutor;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "movement")
public class MovementSequenceExecutor extends MovementHardware implements Autonomous {

    AutonomousExecutor executor;

    @Override
    public void init() {
        super.init();
        executor = new AutonomousExecutor(this);
        executor.init();
    }

    @Override
    public void loop() {
        this.localize();
        boolean done = executor.loop();
        if (done) requestOpModeStop();
    }

    @Override
    public RobotHardware getHardware() {
        return this;
    }

    @Override
    public ActionSequence getActionSequence() {
        return new MovementActionSequence();
    }
}
