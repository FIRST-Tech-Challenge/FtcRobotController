/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

/**
 * Register Op Modes
 */
public class FtcOpModeRegister implements OpModeRegister {

  /**
   * The Op Mode Manager will call this method when it wants a list of all
   * available op modes. Add your op mode to the list to enable it.
   *
   * @param manager op mode manager
   */
  public void register(OpModeManager manager) {

    /*
     * register your op modes here.
     * The first parameter is the name of the op mode
     * The second parameter is the op mode class property
     *
     * If two or more op modes are registered with the same name, the app will display an error.
     */


    /*
     * The following op modes are example op modes provided by QualComm.
     * Uncomment the lines to make the op modes available to the driver station.
     */
    //manager.register("LinearK9TeleOp", LinearK9TeleOp.class);
    //manager.register("LinearIrExample", LinearIrExample.class);
    //manager.register("IrSeekerOp", IrSeekerOp.class);
    //manager.register("CompassCalibration", CompassCalibration.class);
    //manager.register("NxtTeleOp", NxtTeleOp.class);

    /*
     * The NullOp op mode
     */
    manager.register("NullOp", NullOp.class);


    /*
     * The following example op modes are designed to work with a K9-style robot.
     *  - K9TeleOp is a simple driver controlled program.
     *  - K9IrSeeker uses a legacy IR seeker V2 sensor to follow a beacon.
     *  - K9Line uses a legacy LEGO NXT light sensor to follow a white line.
     */

    manager.register("K9TeleOp", K9TeleOp.class);
    manager.register("K9IrSeeker", K9IrSeeker.class);
    manager.register("K9Line", K9Line.class);

    /*
     * The following example op modes are designed to work with a pushbot-style robot.
     *  - PushBotManual is a driver controled (tank drive) op mode.
     *  - PushBotAuto uses the event driven (non linear) OpMode class for autonomous operation.
     *  - PushBotDriveTouch uses the LinearOpMode class and shows how to autonomously drive if a button is not pressed.
     *  - PushBotIrSeek uses the LinearOpMode class and shows how to track an IR beacon.
     *  - PushBotSquare uses the LinearOpMOde class and shows how to drive in a square pattern autonomously.
     */

    manager.register("PushBotManual", PushBotManual.class);
    manager.register("PushBotAuto", PushBotAuto.class);
    manager.register("PushBotDriveTouch", PushBotDriveTouch.class);
    manager.register("PushBotIrSeek", PushBotIrSeek.class);
    manager.register("PushBotSquare", PushBotSquare.class);

  }
}
