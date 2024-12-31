/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;

public class RunCommand extends BTCommand {

    protected final Runnable m_toRun;

    @Override
    public void execute() {
        m_toRun.run();
    }


    /**
     * Creates a new RunCommand.  The Runnable will be run continuously until the command
     * ends.  Does not run when disabled.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems to require
     */
    public RunCommand(@NonNull Runnable toRun, Subsystem... requirements) {
        m_toRun = toRun;

        addRequirements(requirements);
    }
}