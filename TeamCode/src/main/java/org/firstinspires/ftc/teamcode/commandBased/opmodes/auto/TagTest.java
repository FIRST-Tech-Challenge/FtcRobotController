package org.firstinspires.ftc.teamcode.commandBased.opmodes.auto;

import org.firstinspires.ftc.teamcode.commandBased.opmodes.AutoOpMode;

public class TagTest extends AutoOpMode {

    @Override
    public void initialize() {
        super.initialize();
        updateCurrentTag();
    }

    @Override
    public void run() {
        super.run();
        tad("tagPos", tagPos);
    }

}
