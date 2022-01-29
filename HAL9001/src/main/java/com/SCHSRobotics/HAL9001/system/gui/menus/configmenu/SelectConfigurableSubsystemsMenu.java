package com.SCHSRobotics.HAL9001.system.gui.menus.configmenu;

import com.SCHSRobotics.HAL9001.system.config.ConfigSelectionMode;
import com.SCHSRobotics.HAL9001.system.config.HALConfig;
import com.SCHSRobotics.HAL9001.system.gui.DynamicSelectionZone;
import com.SCHSRobotics.HAL9001.system.gui.HALMenu;
import com.SCHSRobotics.HAL9001.system.gui.Payload;
import com.SCHSRobotics.HAL9001.system.gui.event.DataPacket;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EntireViewButton;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.ViewButton;
import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.exceptions.HALConfigException;
import com.SCHSRobotics.HAL9001.util.misc.HALFileUtil;
import com.SCHSRobotics.HAL9001.util.misc.UniqueID;

import org.jetbrains.annotations.NotNull;

import java.util.Set;

/**
 * A menu used to select a specific subsystem to configure out a list of possible configurable subsystems.
 *
 * Creation Date: 9/11/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 *
 * @see ConfigConstants
 * @see ConfigSelectionMenu
 * @see ConfigStartingMenu
 * @see ConfigureSubSystemMenu
 * @see HALMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @see ViewButton
 * @see EntireViewButton
 * @see Payload
 * @see HALConfig
 * @see DynamicSelectionZone
 */
@DynamicSelectionZone(pattern = {true})
public class SelectConfigurableSubsystemsMenu extends HALMenu {
    //Constant UniqueIds for the selected subsystem and the local version of the HAL config being modified by the config.
    protected static final UniqueID SELECTED_SUBSYSTEM_ID = new UniqueID("selected subsystem"), LOCAL_CONFIG_ID = new UniqueID("local config");

    //The filepath to the specific robot folder associated with the currently running robot.
    private final String robotFilepath;
    //The name of the config file.
    private String configFilename;
    //The selection mode of the config.
    private ConfigSelectionMode selectionMode;
    //The local version of the config being modified by the menu.
    private HALConfig localConfig;

    /**
     * The constructor for SelectConfigurableSubsystemsMenu.
     *
     * @param payload The payload passed to this menu.
     * @throws HALConfigException Throws this exception when the payload does not contain the robot filepath or the config file name.
     * @see ConfigConstants
     * @see HALConfig
     * @see com.SCHSRobotics.HAL9001.system.robot.Robot
     */
    public SelectConfigurableSubsystemsMenu(Payload payload) {
        super(payload);

        ExceptionChecker.assertTrue(payload.idPresent(ConfigConstants.ROBOT_FILEPATH_ID), new HALConfigException("Must provide robot filepath."));
        ExceptionChecker.assertTrue(payload.idPresent(ConfigConstants.CONFIG_FILE_NAME_ID), new HALConfigException("Must provide config filename."));

        robotFilepath = payload.get(ConfigConstants.ROBOT_FILEPATH_ID);
        selectionMode = payload.idPresent(ConfigConstants.SELECTION_MODE_ID) ? payload.get(ConfigConstants.SELECTION_MODE_ID) : ConfigSelectionMode.TELEOP;
        configFilename = payload.get(ConfigConstants.CONFIG_FILE_NAME_ID);

        String configFilepath = robotFilepath + selectionMode.filepathExtension + '/' + configFilename.replace(' ', '_') + ConfigConstants.CONFIG_FILE_EXTENSION;

        //If the entered filename doesn't equal the metadata filename and it already exists, read the config file in to get the stored information, otherwise set the local config to a copy of the default config.
        if (!configFilename.replace(' ', '_').equals(ConfigConstants.CONFIG_METADATA_FILENAME) && HALFileUtil.fileExists(configFilepath)) {
            localConfig = HALConfig.readConfig(selectionMode, configFilepath);
        } else {
            localConfig = HALConfig.getDefaultConfig();
        }

        payload.add(LOCAL_CONFIG_ID, localConfig);
    }

    /**
     * The SelectConfigurableSubsystemsMenu's init function.
     *
     * @param payload The payload passed to this menu.
     * @throws HALConfigException Throws this exception when there is no local config in the payload or there is no config file name in the payload.
     * @see ConfigConstants
     * @see ConfigSelectionMenu
     * @see ConfigStartingMenu
     * @see ConfigureSubSystemMenu
     * @see HALMenu
     * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
     * @see ViewButton
     * @see EntireViewButton
     * @see Payload
     * @see HALConfig
     */
    @Override
    protected void init(@NotNull Payload payload) {
        ExceptionChecker.assertTrue(payload.idPresent(LOCAL_CONFIG_ID), new HALConfigException("Local config not present in payload, this should not be possible."));
        ExceptionChecker.assertTrue(payload.idPresent(ConfigConstants.CONFIG_FILE_NAME_ID), new HALConfigException("Must provide config filename."));

        localConfig = payload.get(LOCAL_CONFIG_ID);
        configFilename = payload.get(ConfigConstants.CONFIG_FILE_NAME_ID);

        //If you entered the name of the robot metadata file to try and trick the system, go back and try again :/
        if (configFilename.replace(' ', '_').equals(ConfigConstants.CONFIG_METADATA_FILENAME)) {
            payload.remove(ConfigConstants.CONFIG_FILE_NAME_ID);
            gui.back(payload);
        }

        String configFilepath = robotFilepath+selectionMode.filepathExtension+'/'+configFilename.replace(' ','_')+ConfigConstants.CONFIG_FILE_EXTENSION;

        //Forward and back buttons.
        addItem(new EntireViewButton()
                .onClick(payload.get(ConfigConstants.BACK_BUTTON_ID), (DataPacket packet) -> {
                    payload.remove(ConfigConstants.CONFIG_FILE_NAME_ID);
                    gui.back(payload);
                })
                .onClick(payload.get(ConfigConstants.FORWARD_BUTTON_ID), (DataPacket packet) -> gui.forward(payload)));

        //Add buttons for all configurable subsystems.
        Set<String> subsystemNames = localConfig.getSubsystemNames();
        for (String subsystemName : subsystemNames) {
            addItem(new ViewButton(ConfigConstants.OPTION_PREFIX + subsystemName)
                    .onClick(payload.get(ConfigConstants.SELECT_BUTTON_ID), (DataPacket packet) -> {
                        payload.add(SELECTED_SUBSYSTEM_ID, subsystemName);
                        gui.inflate(new ConfigureSubSystemMenu(), payload);
                    }));
        }

        //Add a "save" button to save the config to a file.
        addItem(new ViewButton(ConfigConstants.OPTION_PREFIX + "Save")
                .onClick(payload.get(ConfigConstants.SELECT_BUTTON_ID), (DataPacket packet) -> {
                    HALConfig.saveConfig(selectionMode, localConfig, configFilepath);
                    payload.remove(ConfigConstants.CONFIG_FILE_NAME_ID);
                    gui.inflate(new ConfigStartingMenu(payload));
                }));
    }
}