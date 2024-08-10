/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.arcrobotics.ftclib.command;

/**
 * Class that holds scheduling state for a command.  Used internally by the
 * {@link CommandScheduler}.
 *
 * <p>
 * <i>Has been <u>grossly</u> oversimplified compared to that of WPILib</i>
 * </p>
 *
 * @author Jackson
 */
class CommandState {

    // Whether or not it is interruptible.
    private final boolean m_interruptible;


    CommandState(boolean interruptible) {
        m_interruptible = interruptible;
    }

    boolean isInterruptible() {
        return m_interruptible;
    }

}