/*
 * Copyright (c) 2015 Robert Atkinson
 *
 *    Ported from the Swerve library by Craig MacFarlane
 *    Based upon contributions and original idea by dmssargent.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Robert Atkinson, Craig MacFarlane nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.qualcomm.robotcore.eventloop.opmode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

/**
 * OpModeManager instances are used to register OpModes for use.
 * see AnnotatedOpModeManager
 * @see OpModeRegistrar
 */
public interface OpModeManager
{
    /**
     * Registers a class for display on the driver station and availability for game play.
     * New instances of this class will be created as needed.
     *
     * @param name          the name to show on the driver station menu
     * @param opModeClass   the class of the OpMode to create
     */
    void register(String name, Class<? extends OpMode> opModeClass);

    /**
     * Registers a class for display on the driver station and availability for game play.
     * New instances of this class will be created as needed.
     *
     * @param name          metadata regarding the class, including the name to show on the DS menu
     * @param opModeClass   the class of the OpMode to create
     */
    void register(OpModeMeta name, Class<? extends OpMode> opModeClass);

    /**
     * Register an *instance* of a class for display on the driver station and availability
     * for game play. You won't likely use this method very often.
     *
     * @param name              the name to show on the driver station menu
     * @param opModeInstance    the object instance to use for that menu item
     */
    void register(String name, OpMode opModeInstance);

    /**
     * Register an *instance* of a class for display on the driver station and availability
     * for game play. You won't likely use this method very often.
     *
     * @param name              metadata regarding the OpMode, including the name to show on the driver station menu
     * @param opModeInstance    the object instance to use for that menu item
     */
    void register(OpModeMeta name, OpMode opModeInstance);

    /**
     * DEFAULT_OP_MODE_NAME is the (non-localized) name of the default OpMode, the one that
     * automatically runs whenever no user OpMode is running.
     */
    String DEFAULT_OP_MODE_NAME = "$Stop$Robot$";
}
