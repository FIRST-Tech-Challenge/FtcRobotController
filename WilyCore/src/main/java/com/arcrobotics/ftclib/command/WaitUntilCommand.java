/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.arcrobotics.ftclib.command;

import java.util.function.BooleanSupplier;

/**
 * A command that does nothing but ends after a specified condition. Useful for
 * CommandGroups.
 *
 * @author Jackson
 */
public class WaitUntilCommand extends CommandBase {

    private final BooleanSupplier m_condition;

    /**
     * Creates a new WaitUntilCommand that ends after a given condition becomes true.
     *
     * @param condition the condition to determine when to end
     */
    public WaitUntilCommand(BooleanSupplier condition) {
        m_condition = condition;
    }

    @Override
    public boolean isFinished() {
        return m_condition.getAsBoolean();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}