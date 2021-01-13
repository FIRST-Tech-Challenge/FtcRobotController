package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.teamcode.action.DetectRingsAction;
import org.firstinspires.ftc.teamcode.action.IfActionResult;
import org.firstinspires.ftc.teamcode.action.WaitForeverAction;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;

public class VisionActionSequence extends ActionSequence {

    DetectRingsAction detectRingsAction = new DetectRingsAction(2000);

    public VisionActionSequence() {
        addAction(detectRingsAction);
        addAction(new IfActionResult(
                detectRingsAction,
                DetectRingsAction.DetectRingsResult.QUAD,
                new WaitForeverAction(),
                null));
    }
}