package com.SCHSRobotics.HAL9001.system.config;

/**
 * An enum denoting the mode of the config system.
 * <p>
 * Creation Date: 9/29/20
 *
 * @author Cole Savage, Level Up
 * @version 2.0.0
 * @see HALConfig
 * @see com.SCHSRobotics.HAL9001.system.gui.menus.configmenu.ConfigStartingMenu
 * @since 1.1.0
 */
public enum ConfigSelectionMode {
    AUTONOMOUS("/autonomous"), TELEOP("/teleop");

    /**
     * The filepath extension to the robot filepath. This will allow the config system to enter into the appropriate subfolders for autonomous and teleop
     */
    public final String filepathExtension;

    /**
     * The constructor for the ConfigSelectionMode enum.
     *
     * @param filepathExtension The filepath extension for the given mode.
     */
    ConfigSelectionMode(String filepathExtension) {
        this.filepathExtension = filepathExtension;
    }
}