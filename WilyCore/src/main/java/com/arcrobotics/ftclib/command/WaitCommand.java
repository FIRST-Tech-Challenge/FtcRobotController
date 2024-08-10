/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.arcrobotics.ftclib.command;

import com.arcrobotics.ftclib.util.Timing.Timer;

import java.util.concurrent.TimeUnit;

/**
 * A command that does nothing but takes a specified amount of time to finish. Useful for
 * CommandGroups. Can also be subclassed to make a command with an internal {@link Timer}.
 *
 * @author Jackson
 */
public class WaitCommand extends CommandBase {

    protected Timer m_timer;

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param millis the time to wait, in milliseconds
     */
    public WaitCommand(long millis) {
        m_timer = new Timer(millis, TimeUnit.MILLISECONDS);
        setName(m_name + ": " + millis + " milliseconds");
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.pause();
    }

    @Override
    public boolean isFinished() {
        return m_timer.done();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
