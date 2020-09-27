package org.firstinspires.ftc.teamcode.playmaker;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

/**
 * Created by djfigs1 on 11/18/16.
 */
public interface Action {

    /**
     * Function called when the action is first executed
     * @param hardware
     */
    void init(RobotHardware hardware);

    /**
     * Function that is called for every iteration of the OpMode controllerLoop
     * @param hardware
     * @return Return true when the action is complete.
     */
    boolean doAction(RobotHardware hardware);

    Double progress();

    String progressString();


}
