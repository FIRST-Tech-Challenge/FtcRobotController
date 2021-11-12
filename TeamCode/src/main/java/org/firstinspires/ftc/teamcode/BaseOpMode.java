package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class is the base for handling all robot movements.
 * @author aryansinha
 */
public abstract class BaseOpMode extends LinearOpMode {
    /**
     * Returns the underlying robot.
     * @return The underlying robot.
     */
    public abstract Hardware2 getRobot();
}
