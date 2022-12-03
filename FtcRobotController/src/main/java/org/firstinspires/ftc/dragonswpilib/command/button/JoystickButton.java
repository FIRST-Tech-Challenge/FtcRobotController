// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.dragonswpilib.command.button;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.GenericHID;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("deprecation")
public class JoystickButton extends Button {

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickButton(Gamepad gamepad, int buttonNumber) {
    //super(() -> joystick.getRawButton(buttonNumber));
    //requireNonNullParam(joystick, "joystick", "JoystickButton");
    super(() -> new GenericHID(gamepad).getRawButton(buttonNumber));
  }

}
