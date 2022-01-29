package com.SCHSRobotics.HAL9001.system.gui.menus.configmenu;

import com.SCHSRobotics.HAL9001.system.config.ConfigSelectionMode;
import com.SCHSRobotics.HAL9001.system.config.HALConfig;
import com.SCHSRobotics.HAL9001.system.gui.DynamicSelectionZone;
import com.SCHSRobotics.HAL9001.system.gui.HALMenu;
import com.SCHSRobotics.HAL9001.system.gui.Payload;
import com.SCHSRobotics.HAL9001.system.gui.event.DataPacket;
import com.SCHSRobotics.HAL9001.system.gui.menus.TextSelectionMenu;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EntireViewButton;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.ViewButton;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.exceptions.HALConfigException;
import com.SCHSRobotics.HAL9001.util.misc.HALFileUtil;

import org.jetbrains.annotations.NotNull;

/**
 * The starting screen for the config system.
 *
 * Creation Date: 9/11/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 *
 * @see ConfigConstants
 * @see ConfigSelectionMenu
 * @see TextSelectionMenu
 * @see ConfigureSubSystemMenu
 * @see HALMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @see ViewButton
 * @see EntireViewButton
 * @see Payload
 * @see DynamicSelectionZone
 */
@DynamicSelectionZone(pattern = {true})
public class ConfigStartingMenu extends HALMenu {

    /**
     * The filepath to the specific folder where the config files for this robot are stored.
     */
    private final String robotFilepath;

    /**
     * The constructor for the initial config menu.
     *
     * @param payload A payload object carrying data of use to the menu. This payload should contain a robot folder filepath, a selection mode,
     *                all necessary config controls, and, optionally, the "stand alone" and "save to autorun" options
     * @throws HALConfigException Throws this exception if the robot filepath is not provided via the payload.
     * @see Payload
     * @see HALMenu
     */
    public ConfigStartingMenu(Payload payload) {
        super(payload);
        ExceptionChecker.assertTrue(payload.idPresent(ConfigConstants.ROBOT_FILEPATH_ID), new HALConfigException("Must provide robot filepath."));
        robotFilepath = payload.get(ConfigConstants.ROBOT_FILEPATH_ID);
        setupControls(payload);
    }

    /**
     * Ensures that controls for the config system exist in the payload. If a control is not already in the payload, it adds the default value.
     *
     * @param payload The payload that will contain the config system controls.
     * @see Payload
     * @see Button
     */
    private static void setupControls(@NotNull Payload payload) {
        if (!payload.idPresent(ConfigConstants.BACK_BUTTON_ID) || !(payload.get(ConfigConstants.BACK_BUTTON_ID) instanceof Button<?>) || !((Button<?>) payload.get(ConfigConstants.BACK_BUTTON_ID)).isBoolean()) {
            payload.add(ConfigConstants.BACK_BUTTON_ID, ConfigConstants.DEFAULT_BACK_BUTTON);
        }
        if (!payload.idPresent(ConfigConstants.FORWARD_BUTTON_ID) || !(payload.get(ConfigConstants.FORWARD_BUTTON_ID) instanceof Button<?>) || !((Button<?>) payload.get(ConfigConstants.FORWARD_BUTTON_ID)).isBoolean()) {
            payload.add(ConfigConstants.FORWARD_BUTTON_ID, ConfigConstants.DEFAULT_FORWARD_BUTTON);
        }
        if (!payload.idPresent(ConfigConstants.TEXT_ENTRY_LEFT_ID) || !(payload.get(ConfigConstants.TEXT_ENTRY_LEFT_ID) instanceof Button<?>) || !((Button<?>) payload.get(ConfigConstants.TEXT_ENTRY_LEFT_ID)).isBoolean()) {
            payload.add(ConfigConstants.TEXT_ENTRY_LEFT_ID, ConfigConstants.DEFAULT_TEXT_ENTRY_LEFT);
        }
        if (!payload.idPresent(ConfigConstants.TEXT_ENTRY_RIGHT_ID) || !(payload.get(ConfigConstants.TEXT_ENTRY_RIGHT_ID) instanceof Button<?>) || !((Button<?>) payload.get(ConfigConstants.TEXT_ENTRY_RIGHT_ID)).isBoolean()) {
            payload.add(ConfigConstants.TEXT_ENTRY_RIGHT_ID, ConfigConstants.DEFAULT_TEXT_ENTRY_RIGHT);
        }
        if (!payload.idPresent(ConfigConstants.SELECT_BUTTON_ID) || !(payload.get(ConfigConstants.SELECT_BUTTON_ID) instanceof Button<?>) || !((Button<?>) payload.get(ConfigConstants.SELECT_BUTTON_ID)).isBoolean()) {
            payload.add(ConfigConstants.SELECT_BUTTON_ID, ConfigConstants.DEFAULT_SELECT_BUTTON);
        }
        if (!payload.idPresent(ConfigConstants.REVERSE_SELECT_BUTTON_ID) || !(payload.get(ConfigConstants.REVERSE_SELECT_BUTTON_ID) instanceof Button<?>) || !((Button<?>) payload.get(ConfigConstants.REVERSE_SELECT_BUTTON_ID)).isBoolean()) {
            payload.add(ConfigConstants.REVERSE_SELECT_BUTTON_ID, ConfigConstants.DEFAULT_REVERSE_SELECT_BUTTON);
        }
        if (!payload.idPresent(ConfigConstants.CHANGE_GAMEPAD_BUTTON_ID) || !(payload.get(ConfigConstants.CHANGE_GAMEPAD_BUTTON_ID) instanceof Button<?>) || !((Button<?>) payload.get(ConfigConstants.CHANGE_GAMEPAD_BUTTON_ID)).isBoolean()) {
            payload.add(ConfigConstants.CHANGE_GAMEPAD_BUTTON_ID, ConfigConstants.DEFAULT_CHANGE_GAMEPAD_BUTTON);
        }
    }

    /**
     * The ConfigStartingMenu's init function.
     *
     * @param payload The payload passed to this menu.
     * @see ConfigConstants
     * @see ConfigSelectionMenu
     * @see TextSelectionMenu
     * @see ConfigureSubSystemMenu
     * @see HALMenu
     * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
     * @see ViewButton
     * @see EntireViewButton
     * @see Payload
     */
    @Override
    protected void init(Payload payload) {
        //Extracts data from the payload
        ConfigSelectionMode selectionMode = payload.idPresent(ConfigConstants.SELECTION_MODE_ID) ? payload.get(ConfigConstants.SELECTION_MODE_ID) : ConfigSelectionMode.TELEOP;
        boolean isStandalone = payload.idPresent(ConfigConstants.STANDALONE_MODE_ID) ? payload.get(ConfigConstants.STANDALONE_MODE_ID) : false;
        boolean saveToAutorun = payload.idPresent(ConfigConstants.SAVE_TO_AUTORUN_ID) ? payload.get(ConfigConstants.SAVE_TO_AUTORUN_ID) : false;

        //If the menu should be autorunning a config file.
        if (!isStandalone && !saveToAutorun && selectionMode == ConfigSelectionMode.TELEOP) {
            //Gets the path to the config file to auto-run
            String autorunFilepath = robotFilepath + ConfigSelectionMode.TELEOP.filepathExtension + '/' + ConfigConstants.CONFIG_METADATA_FILENAME + ConfigConstants.CONFIG_FILE_EXTENSION;
            String autorunConfigFilepath = HALFileUtil.readFile(autorunFilepath);

            //If the specified config file exists, load it into the config and inflate the autorun screen.
            if(HALFileUtil.fileExists(autorunConfigFilepath)) {
                HALConfig config = HALConfig.readConfig(selectionMode, autorunConfigFilepath);
                HALConfig.updateGlobalConfig(config);

                int nameStartIdx = autorunConfigFilepath.lastIndexOf('/') + 1;
                int nameEndIdx = autorunConfigFilepath.indexOf(ConfigConstants.CONFIG_FILE_EXTENSION);
                String configFileName = autorunConfigFilepath.substring(nameStartIdx, nameEndIdx);
                payload.add(ConfigConstants.CONFIG_FILE_NAME_ID, configFileName);
                gui.inflate(new AutorunScreen(), payload);
            }
            //Otherwise, restart the menu in standalone mode and store the selected file to the autorun file.
            else {
                payload.add(ConfigConstants.STANDALONE_MODE_ID, true);
                payload.add(ConfigConstants.SAVE_TO_AUTORUN_ID, true);
                gui.inflate(new ConfigStartingMenu(payload));
            }
        }
        //If the payload contains a config filename as a relic from a previous menu, delete it and the corresponding config file if there is one, they shouldn't be here at this point.
        else if(payload.idPresent(ConfigConstants.CONFIG_FILE_NAME_ID)) {
            String filepathToDelete = robotFilepath+selectionMode.filepathExtension+'/'+payload.get(ConfigConstants.CONFIG_FILE_NAME_ID)+ConfigConstants.CONFIG_FILE_EXTENSION;
            HALFileUtil.deleteFile(filepathToDelete);
            payload.remove(ConfigConstants.CONFIG_FILE_NAME_ID);
        }

        String[] configFilenames = HALFileUtil.getAllFileNames(robotFilepath + selectionMode.filepathExtension, ConfigConstants.CONFIG_METADATA_FILENAME);
        //Forward button.
        addItem(new EntireViewButton()
                .onClick(payload.get(ConfigConstants.FORWARD_BUTTON_ID), (DataPacket packet) -> gui.forward(payload)));

        //Create new config button.
        addItem(new ViewButton(ConfigConstants.OPTION_PREFIX+"New Config")
                .onClick(payload.get(ConfigConstants.SELECT_BUTTON_ID), (DataPacket packet) -> {
                    payload.add(TextSelectionMenu.NEXT_MENU_ID, SelectConfigurableSubsystemsMenu.class);
                    gui.inflate(new TextSelectionMenu(payload));
                }));

        //Edit existing config button.
        addItem(new ViewButton(ConfigConstants.OPTION_PREFIX+"Edit Config")
                .onClick(payload.get(ConfigConstants.SELECT_BUTTON_ID), (DataPacket packet) -> {
                    if(configFilenames.length > 0) {
                        payload.add(ConfigConstants.NEXT_MENU_ID, SelectConfigurableSubsystemsMenu.class);
                        gui.inflate(new ConfigSelectionMenu(payload));
                    }
                }));

        //Delete existing config button.
        addItem(new ViewButton(ConfigConstants.OPTION_PREFIX+"Delete Config")
                .onClick(payload.get(ConfigConstants.SELECT_BUTTON_ID), (DataPacket packet) -> {
                    if(configFilenames.length > 0) {
                        payload.add(ConfigConstants.NEXT_MENU_ID, ConfigStartingMenu.class);
                        gui.inflate(new ConfigSelectionMenu(payload));
                    }
                }));

        //Adds all config file options to the screen.
        for(String configFilename : configFilenames) {
            addItem(new ViewButton(ConfigConstants.OPTION_PREFIX + configFilename)
                    .onClick(payload.get(ConfigConstants.SELECT_BUTTON_ID), (DataPacket packet) -> {
                        //Loads the selected config file.
                        String configFilepath = robotFilepath + selectionMode.filepathExtension + '/' + configFilename + ConfigConstants.CONFIG_FILE_EXTENSION;
                        HALConfig config = HALConfig.readConfig(selectionMode, configFilepath);
                        HALConfig.updateGlobalConfig(config);

                        //Saves file to autorun if "save to autorun" mode is on.
                        if(saveToAutorun) {
                            String autorunFilepath = robotFilepath + ConfigSelectionMode.TELEOP.filepathExtension + '/' + ConfigConstants.CONFIG_METADATA_FILENAME + ConfigConstants.CONFIG_FILE_EXTENSION;
                            HALFileUtil.save(autorunFilepath, configFilepath);
                        }

                        //If it is not standalone and autonomous, reset the payload and ask the user to select a file for teleop.
                        if(!isStandalone && selectionMode == ConfigSelectionMode.AUTONOMOUS) {
                            Payload newPayload = new Payload()
                                    .copyFrom(payload,
                                            ConfigConstants.BACK_BUTTON_ID,
                                            ConfigConstants.FORWARD_BUTTON_ID,
                                            ConfigConstants.TEXT_ENTRY_LEFT_ID,
                                            ConfigConstants.TEXT_ENTRY_RIGHT_ID,
                                            ConfigConstants.SELECT_BUTTON_ID,
                                            ConfigConstants.REVERSE_SELECT_BUTTON_ID,
                                            ConfigConstants.CHANGE_GAMEPAD_BUTTON_ID,
                                            ConfigConstants.ROBOT_FILEPATH_ID)
                                    .add(ConfigConstants.SELECTION_MODE_ID, ConfigSelectionMode.TELEOP)
                                    .add(ConfigConstants.STANDALONE_MODE_ID, true)
                                    .add(ConfigConstants.SAVE_TO_AUTORUN_ID, true);
                            gui.inflate(new ConfigStartingMenu(newPayload));
                        } else {
                            gui.removeCurrentStack();
                        }
                    }));
        }
    }
}