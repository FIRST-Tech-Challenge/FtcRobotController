package org.firstinspires.ftc.teamcode.commandBased.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.opmodes.AutoOpMode;

@TeleOp
public class TagTest extends AutoOpMode {

    @Override
    public void initialize() {
        super.initialize();
        updateCurrentTag();
    }

    @Override
    public void run() {
        super.run();
        determinePathFromTag();
        tad("tagPos", tagPos);
    }

}
