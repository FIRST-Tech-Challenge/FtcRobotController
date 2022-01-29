package com.SCHSRobotics.HAL9001.system.gui.menus.configmenu;

import com.SCHSRobotics.HAL9001.system.config.ConfigSelectionMode;
import com.SCHSRobotics.HAL9001.system.gui.DynamicSelectionZone;
import com.SCHSRobotics.HAL9001.system.gui.HALMenu;
import com.SCHSRobotics.HAL9001.system.gui.Payload;
import com.SCHSRobotics.HAL9001.system.gui.event.DataPacket;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EntireViewButton;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.ViewButton;
import com.SCHSRobotics.HAL9001.util.exceptions.DumpsterFireException;
import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.misc.HALFileUtil;

import org.jetbrains.annotations.NotNull;

/**
 * A menu used to select a specific configuration file out of a list of all available configuration files.
 *
 * Creation Date: 9/11/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 *
 * @see ConfigConstants
 * @see ConfigStartingMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.menus.TextSelectionMenu
 * @see ConfigureSubSystemMenu
 * @see HALMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @see ViewButton
 * @see EntireViewButton
 * @see Payload
 * @see DynamicSelectionZone
 * @see HALFileUtil
 */
@DynamicSelectionZone(pattern = {true})
public class ConfigSelectionMenu extends HALMenu {

    /**
     * The filepath to the specific folder where the config files for this robot are stored.
     */
    private final String robotFilepath;

    /**
     * A constructor for the ConfigSelectionMenu.
     *
     * @param payload A payload object carrying data of use to the menu. This payload should contain a robot folder filepath, a selection mode,
     *                all necessary config controls, and, optionally, the class of the menu that should be inflated after a config file has been selected.
     */
    public ConfigSelectionMenu(Payload payload) {
        super(payload);

        ExceptionChecker.assertTrue(payload.idPresent(ConfigConstants.ROBOT_FILEPATH_ID), new DumpsterFireException("Must provide robot filepath."));
        robotFilepath = payload.get(ConfigConstants.ROBOT_FILEPATH_ID);
    }

    /**
     * The ConfigSelectionMenu's init function.
     *
     * @param payload The payload passed to this menu.
     * @see ConfigConstants
     * @see ConfigStartingMenu
     * @see com.SCHSRobotics.HAL9001.system.gui.menus.TextSelectionMenu
     * @see ConfigureSubSystemMenu
     * @see HALMenu
     * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
     * @see ViewButton
     * @see EntireViewButton
     * @see Payload
     */
    @Override
    protected void init(@NotNull Payload payload) {
        ConfigSelectionMode selectionMode = payload.idPresent(ConfigConstants.SELECTION_MODE_ID) ? payload.get(ConfigConstants.SELECTION_MODE_ID) : ConfigSelectionMode.TELEOP;
        Class<HALMenu> nextMenu = payload.idPresent(ConfigConstants.NEXT_MENU_ID) ? payload.get(ConfigConstants.NEXT_MENU_ID) : null;

        //Forward and back buttons
        addItem(new EntireViewButton()
                .onClick(payload.get(ConfigConstants.BACK_BUTTON_ID), (DataPacket packet) -> gui.back(payload))
                .onClick(payload.get(ConfigConstants.FORWARD_BUTTON_ID), (DataPacket packet) -> gui.forward(payload)));

        //Prints all valid config files as selectable options
        String[] configFilenames = HALFileUtil.getAllFileNames(robotFilepath + selectionMode.filepathExtension, ConfigConstants.CONFIG_METADATA_FILENAME);
        for(String configFilename : configFilenames) {
            addItem(new ViewButton(ConfigConstants.OPTION_PREFIX+configFilename)
                    .onClick(payload.get(ConfigConstants.SELECT_BUTTON_ID), (DataPacket packet) -> {
                        payload.add(ConfigConstants.CONFIG_FILE_NAME_ID, configFilename);
                        //If no next menu is provided in the payload, simply return to the last menu (the menu that inflated this one), otherwise inflate that new menu (note: calls the constructor of given menu class)
                        if (nextMenu == null) gui.back(payload);
                        else gui.inflate(nextMenu, payload);
                    }));
        }
    }
}