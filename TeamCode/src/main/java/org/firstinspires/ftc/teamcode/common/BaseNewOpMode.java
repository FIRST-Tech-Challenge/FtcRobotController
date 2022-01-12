package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.HardwareNew;

/**
 * This class is the base for handling all robot movements.
 * @author aryansinha
 */
public abstract class BaseNewOpMode extends LinearOpMode {
    /**
     * Returns the underlying robot.
     *
     * @return The underlying robot.
     */
    public abstract HardwareNew getRobot();
}
