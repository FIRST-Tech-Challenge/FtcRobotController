package org.firstinspires.ftc.teamcode.opmodes.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.AutoOpModeBase;

@Autonomous(name = "Auto Crash test")
public class Test extends AutoOpModeBase {

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void executeOpMode() {
        pivot.MoveToIntakeInAuto();
    }

}