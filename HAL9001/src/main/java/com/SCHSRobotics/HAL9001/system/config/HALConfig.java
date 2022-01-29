package com.SCHSRobotics.HAL9001.system.config;

import android.util.Log;

import com.SCHSRobotics.HAL9001.system.robot.HALProgram;
import com.SCHSRobotics.HAL9001.system.robot.LinkTo;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.exceptions.InvalidLinkException;
import com.SCHSRobotics.HAL9001.util.exceptions.OpModeAnnotationNotPresentException;
import com.SCHSRobotics.HAL9001.util.math.FakeNumpy;
import com.SCHSRobotics.HAL9001.util.math.datastructures.BidirectionalMap;
import com.SCHSRobotics.HAL9001.util.misc.HALFileUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * A class for managing HAL's config system.
 *
 * Creation Date: 3/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 * @see AutonomousConfig
 * @see TeleopConfig
 * @see ProgramOptions
 * @see ConfigLabel
 * @see DisableSubSystem
 * @see LinkTo
 * @see ConfigParam
 * @see ConfigData
 * @see ConfigSelectionMode
 * @see com.SCHSRobotics.HAL9001.system.robot.Robot
 * @see com.SCHSRobotics.HAL9001.system.gui.menus.configmenu.ConfigStartingMenu
 */
public final class HALConfig {
    //The tag used to identify this class in the log.
    private static final String LOGGING_TAG = "HAL Config";

    //The static global instance of the HAL config, which is used as the central database for the config.
    private static HALConfig GLOBAL_INSTANCE = new HALConfig();
    //The default config database values.
    private static HALConfig DEFAULT_CONFIG = new HALConfig();

    //Hashmaps that map the id of subsystems to their corresponding configuration settings.
    private Map<String, List<ConfigParam>> autonomousConfig = new HashMap<>(), teleopConfig = new HashMap<>();
    //maps name of a subsystem to its id (reverse is id to name)
    private BidirectionalMap<String, String> subsystemIdLookup = new BidirectionalMap<>();
    //Maps subsystem objects to their id and vice versa.
    private BidirectionalMap<SubSystem, String> subSystemNameLookup = new BidirectionalMap<>();

    /**
     * A public default constructor for the HAL config class.
     */
    public HALConfig() {
    }

    /**
     * A private constructor for the HAL config used for cloning purposes.
     *
     * @param sourceForClone The HALConfig object being cloned.
     */
    private HALConfig(@NotNull HALConfig sourceForClone) {
        //Copy the autonomous config.
        for (Map.Entry<String, List<ConfigParam>> configEntry : sourceForClone.autonomousConfig.entrySet()) {
            List<ConfigParam> clonedParams = new ArrayList<>();
            for (ConfigParam param : configEntry.getValue()) {
                clonedParams.add(param.clone());
            }
            autonomousConfig.put(configEntry.getKey(), clonedParams);
        }
        //Copy the teleop config.
        for (Map.Entry<String, List<ConfigParam>> configEntry : sourceForClone.teleopConfig.entrySet()) {
            List<ConfigParam> clonedParams = new ArrayList<>();
            for (ConfigParam param : configEntry.getValue()) {
                clonedParams.add(param.clone());
            }
            teleopConfig.put(configEntry.getKey(), clonedParams);
        }
        //Copy the subsystem name lookup.
        for (Map.Entry<SubSystem, String> subSystemEntry : sourceForClone.subSystemNameLookup.entrySet()) {
            subSystemNameLookup.put(subSystemEntry.getKey(), subSystemEntry.getValue());
        }

        //Copy the subsystem id lookup.
        subsystemIdLookup.putAll(sourceForClone.subsystemIdLookup);
    }

    /**
     * Gets the global instance of the HAL Config.
     *
     * @return The global instance of the HAL Config.
     */
    public static HALConfig getGlobalInstance() {
        return GLOBAL_INSTANCE;
    }

    /**
     * Gets the default config.
     *
     * @return The default config.
     */
    @NotNull
    public static HALConfig getDefaultConfig() {
        return DEFAULT_CONFIG.clone();
    }

    /**
     * Gets the name of a given opmode.
     *
     * @param opModeClass The opmode class object.
     * @return The opmode's given name (via the @TeleOp or @Autonomous annotations).
     * @throws OpModeAnnotationNotPresentException Throws this exception when the given opmode class is not annotated with @TeleOp or @Autonomous.
     * @see Autonomous
     * @see TeleOp
     */
    public static String getOpModeName(@NotNull Class<? extends OpMode> opModeClass) {
        String name;

        if (opModeClass.isAnnotationPresent(TeleOp.class)) {
            TeleOp op = Objects.requireNonNull(opModeClass.getAnnotation(TeleOp.class));
            name = op.name();
        } else if (opModeClass.isAnnotationPresent(Autonomous.class)) {
            Autonomous op = Objects.requireNonNull(opModeClass.getAnnotation(Autonomous.class));
            name = op.name();
        } else {
            throw new OpModeAnnotationNotPresentException("Program " + opModeClass.getSimpleName() + " can't be run without @TeleOp or @Autonomous");
        }
        return name;
    }

    /**
     * Gets whether a given opmode is an autonomous program or a teleop program.
     *
     * @param opModeClass The opmode class object.
     * @return The ConfigMode object associated with the opmode's type.
     * @throws OpModeAnnotationNotPresentException Throws this exception when the given opmode class is not annotated with @TeleOp or @Autonomous.
     * @see Autonomous
     * @see TeleOp
     * @see ConfigSelectionMode
     */
    public static ConfigSelectionMode getOpModeType(@NotNull Class<? extends OpMode> opModeClass) {
        if (opModeClass.isAnnotationPresent(Autonomous.class))
            return ConfigSelectionMode.AUTONOMOUS;
        else if (opModeClass.isAnnotationPresent(TeleOp.class)) return ConfigSelectionMode.TELEOP;
        else
            throw new OpModeAnnotationNotPresentException("Program " + opModeClass.getSimpleName() + " can't be run without @TeleOp or @Autonomous");
    }

    /**
     * Sets the given config as the default config.
     *
     * @param defaultConfig The config to set as default.
     */
    public static void setAsDefault(@NotNull HALConfig defaultConfig) {
        DEFAULT_CONFIG.clearConfig();

        DEFAULT_CONFIG.autonomousConfig.putAll(defaultConfig.autonomousConfig);
        DEFAULT_CONFIG.teleopConfig.putAll(defaultConfig.teleopConfig);
        DEFAULT_CONFIG.subsystemIdLookup.putAll(defaultConfig.subsystemIdLookup);
    }

    /**
     * Sets the global config as the default config.
     */
    public static void setGlobalConfigAsDefault() {
        setAsDefault(GLOBAL_INSTANCE);
    }

    /**
     * Updates the global config with values from the given config.
     *
     * @param config The config being used to update the global config.
     */
    public static void updateGlobalConfig(@NotNull HALConfig config) {
        GLOBAL_INSTANCE.autonomousConfig.clear();
        GLOBAL_INSTANCE.teleopConfig.clear();

        GLOBAL_INSTANCE.autonomousConfig.putAll(config.autonomousConfig);
        GLOBAL_INSTANCE.teleopConfig.putAll(config.teleopConfig);
    }

    /**
     * Saves the current config to a file.
     *
     * @param mode     The mode of the config (autonomous or teleop).
     * @param config   The config to save to the file.
     * @param filepath The filepath specifying where to save the file.
     * @see ConfigSelectionMode
     * @see ConfigParam
     * @see HALFileUtil
     */
    public static void saveConfig(ConfigSelectionMode mode, @NotNull HALConfig config, String filepath) {
        StringBuilder sb = new StringBuilder();
        for (String subsystem : config.getSubsystemNames()) {
            List<ConfigParam> params = config.getConfig(mode, subsystem);
            if (params == null) params = new ArrayList<>();

            for (ConfigParam param : params) {
                sb.append(subsystem);
                sb.append(':');
                sb.append(param.name);
                sb.append(':');
                sb.append(param.currentOption);
                if (param.usesGamepad) {
                    sb.append(':');
                    sb.append(param.currentGamepadOption);
                }
                sb.append("\r\n");
            }
        }

        if (sb.length() > 2) {
            sb.delete(sb.length() - 2, sb.length()); //removes trailing \r\n characters so there isn't a blank line at the end of the file
        }

        HALFileUtil.save(filepath, sb.toString());
    }

    /**
     * Reads a config file into a HALConfig object.
     *
     * @param mode     The mode of the config (autonomous or teleop).
     * @param filepath The filepath specifying where to save the file.
     * @return The HALConfig object that was stored in the file.
     * @see ConfigSelectionMode
     * @see ConfigParam
     * @see HALFileUtil
     */
    @NotNull
    public static HALConfig readConfig(ConfigSelectionMode mode, String filepath) {
        FileInputStream fis;

        HALConfig outputConfig = HALConfig.getDefaultConfig();

        try {
            fis = new FileInputStream(filepath);

            FileReader fReader;
            BufferedReader bufferedReader;

            //TODO: Make this a bit less jank. Right now throwing errors is perfectly fine here because that's how it fixes out of date config files.

            try {
                fReader = new FileReader(fis.getFD());
                bufferedReader = new BufferedReader(fReader);

                int i = 0;
                String lastSubsystem = "\n";
                String line;
                while ((line = bufferedReader.readLine()) != null) {
                    String[] data = line.split(":");
                    if (!data[0].equals(lastSubsystem)) i = 0;
                    List<ConfigParam> params = outputConfig.getConfig(mode, data[0]);
                    if (params != null && params.get(i).name.equals(data[1].trim())) {
                        params.get(i).name = data[1].trim();
                        params.get(i).currentOption = data[2].trim();
                        if (params.get(i).usesGamepad) {
                            params.get(i).currentGamepadOption = data[3].trim();
                        }
                        lastSubsystem = data[0];

                        i++;
                    }
                }

                bufferedReader.close();
                fReader.close();
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                fis.getFD().sync();
                fis.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return outputConfig;
    }

    /**
     * Adds a subsystem to the config.
     *
     * @param name      The subsystem's lookup id in the config.
     * @param subSystem The subsystem to add to the config.
     * @return Whether or not the config settings were successfully added.
     * @see AutonomousConfig
     * @see TeleopConfig
     * @see ConfigLabel
     * @see DisableSubSystem
     * @see com.SCHSRobotics.HAL9001.system.robot.Robot
     * @see SubSystem
     */
    public final boolean addSubSystem(String name, SubSystem subSystem) {
        int i = 1;
        String tempName = name;
        while (teleopConfig.containsKey(tempName) || autonomousConfig.containsKey(tempName)) {
            tempName = name + i;
            i++;
        }

        Class<? extends SubSystem> subSystemClass = subSystem.getClass();

        //Adds a number to the end of the subsystem class name to make the id unique.
        i = 1;
        String className = subSystemClass.getSimpleName();
        String id = className;
        while(subsystemIdLookup.containsValue(id)) {
            id = className + i;
            i++;
        }
        subsystemIdLookup.put(tempName, id);
        subSystemNameLookup.put(subSystem, tempName);

        boolean foundTeleopConfig = false;
        boolean foundAutonomousConfig = false;

        try {
            Method[] methods = subSystemClass.getDeclaredMethods();
            for (Method m : methods) {

                //method must be annotated as TeleopConfig, have no parameters, be public and static, and return an array of config params
                if (!foundTeleopConfig && m.isAnnotationPresent(TeleopConfig.class) && m.getReturnType() == ConfigParam[].class && m.getParameterTypes().length == 0 && Modifier.isStatic(m.getModifiers()) && Modifier.isPublic(m.getModifiers())) {
                    teleopConfig.put(tempName, Arrays.asList((ConfigParam[]) m.invoke(null)));
                    foundTeleopConfig = true;
                }

                //method must be annotated as AutonomousConfig, have no parameters, be public and static, and return an array of config params
                if (!foundAutonomousConfig && m.isAnnotationPresent(AutonomousConfig.class) && m.getReturnType() == ConfigParam[].class && m.getParameterTypes().length == 0 && Modifier.isStatic(m.getModifiers()) && Modifier.isPublic(m.getModifiers())) {
                    autonomousConfig.put(tempName, Arrays.asList((ConfigParam[]) m.invoke(null)));
                    foundAutonomousConfig = true;
                }

                if (foundTeleopConfig && foundAutonomousConfig) break;
            }
        } catch (Throwable e) {
            Log.e(LOGGING_TAG, "Problem loading config for subsystem " + subSystem.getClass().getSimpleName(), e);
        }

        return foundAutonomousConfig || foundTeleopConfig;
    }

    /**
     * Exposed interface for the addOpmode function. Adds an opmode to the config.
     *
     * @param opMode The opmode to add.
     * @throws InvalidLinkException Throws this exception when a destination specified in a LinkTo annotation does not link to a real opmode.
     * @see HALProgram
     * @see OpMode
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     * @see LinkTo
     * @see ProgramOptions
     */
    public final void addOpmode(@NotNull OpMode opMode) {
        addOpmode(opMode, new ArrayList<>());
    }

    /**
     * A private version of addOpmode that recursively adds the config settings for the given opmode and all linked opmodes.
     *
     * @param opMode         The opmode to add.
     * @param visitedOpmodes The list of linked opmodes that have already been added.
     * @throws InvalidLinkException Throws this exception when a destination specified in a LinkTo annotation does not link to a real opmode.
     * @see HALProgram
     * @see OpMode
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     * @see LinkTo
     * @see ProgramOptions
     */
    private void addOpmode(@NotNull OpMode opMode, @NotNull List<String> visitedOpmodes) {

        Class<? extends OpMode> opModeClass = opMode.getClass();

        ConfigSelectionMode programMode = getOpModeType(opModeClass);
        String name = getOpModeName(opModeClass);

        visitedOpmodes.add(name);
        if (opModeClass.isAnnotationPresent(ProgramOptions.class)) {
            ProgramOptions options = Objects.requireNonNull(opModeClass.getAnnotation(ProgramOptions.class));

            List<ConfigParam> settings = extractConfigFromSettings(options);
            if (settings.size() > 0) {
                switch (programMode) {
                    case AUTONOMOUS:
                        if(autonomousConfig.containsKey(name)) {
                            Log.e(LOGGING_TAG, "Autonomous config already has an entry for " + name + '.');
                        }
                        autonomousConfig.put(name, settings);
                        break;
                    case TELEOP:
                        if(teleopConfig.containsKey(name)) {
                            Log.e(LOGGING_TAG, "Teleop config already has an entry for " + name + '.');
                        }
                        teleopConfig.put(name, settings);
                        break;
                }
            }
        }

        //If there is a LinkTo annotation, add the opmode associated with that link to the config as well.
        if (opModeClass.isAnnotationPresent(LinkTo.class) && !visitedOpmodes.contains(name)) {
            LinkTo link = Objects.requireNonNull(opModeClass.getAnnotation(LinkTo.class));

            //Gets the linked opmode from the internal list of registered opmodes.
            OpMode linkedOpmode = RegisteredOpModes.getInstance().getOpMode(link.destination());
            ExceptionChecker.assertNonNull(linkedOpmode, new InvalidLinkException("Link to nonexistent opmode. '" + link.destination() + "' does not exist"));

            if (linkedOpmode instanceof HALProgram) addOpmode(opMode, visitedOpmodes);
        }
    }

    /**
     * Adds an entry to the config in the given mode under the given id.
     *
     * @param mode         The mode of the config (autonomous or teleop)/
     * @param lookup       The id that the config params will be put under.
     * @param configParams The actual config params that will be added to the config.
     * @see ConfigSelectionMode
     * @see ConfigParam
     */
    public final void setConfig(ConfigSelectionMode mode, String lookup, List<ConfigParam> configParams) {
        if (mode == ConfigSelectionMode.TELEOP) teleopConfig.put(lookup, configParams);
        else autonomousConfig.put(lookup, configParams);
    }

    /**
     * A private function used to extract config settings from the ProgramOptions annotation.
     *
     * @param opts The ProgramOptions annotion that config settings will be extracted from.
     * @return A list of config params representing the config options specified in the annotation.
     * @see ProgramOptions
     * @see ConfigParam
     */
    @NotNull
    private List<ConfigParam> extractConfigFromSettings(@NotNull ProgramOptions opts) {
        List<ConfigParam> options = new ArrayList<>();

        //cleanup, removes any duplicated enum classes from options.
        Class<? extends Enum<?>>[] nonDuplicatedSettings = FakeNumpy.removeDuplicates(opts.options());

        if (nonDuplicatedSettings.length > 0) {
            for (Class<? extends Enum<?>> option : nonDuplicatedSettings) {
                Enum<?>[] enums = option.getEnumConstants();
                if (enums.length == 0) continue;
                options.add(new ConfigParam(option.getSimpleName(), enums[0]));
            }
        }
        return options;
    }

    /**
     * Gets the config settings associated with a given subsystem.
     *
     * @param mode      The mode of the config (autonomous or teleop).
     * @param subSystem The subsystem to get the config for.
     * @return The list of config params associated with the given subsystem.
     * @see ConfigSelectionMode
     * @see SubSystem
     */
    @Nullable
    public List<ConfigParam> getConfig(ConfigSelectionMode mode, SubSystem subSystem) {
        return getConfig(mode, subSystemNameLookup.getForward(subSystem));
    }

    /**
     * Gets the config settings associated with a given id.
     *
     * @param mode The mode of the config (autonomous or teleop).
     * @param name The id that the desired config is listed under.
     * @return The list of config params associated with the given id.
     * @see ConfigSelectionMode
     * @see SubSystem
     */
    @Nullable
    public List<ConfigParam> getConfig(ConfigSelectionMode mode, String name) {
        if ((mode == ConfigSelectionMode.AUTONOMOUS && !autonomousConfig.containsKey(name)) || (mode == ConfigSelectionMode.TELEOP && !teleopConfig.containsKey(name))) {
            return null;
        }
        return mode == ConfigSelectionMode.AUTONOMOUS ? autonomousConfig.get(name) : teleopConfig.get(name);
    }

    /**
     * Gets the config settings associated with a given opmode.
     *
     * @param opMode The opmode to get the config for.
     * @return The list of config params associated with the given opmode.
     * @see ConfigSelectionMode
     * @see HALProgram
     * @see OpMode
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     */
    @Nullable
    public List<ConfigParam> getConfig(@NotNull OpMode opMode) {
        Class<? extends OpMode> opModeClass = opMode.getClass();
        return getConfig(getOpModeType(opModeClass), getOpModeName(opModeClass));
    }

    /**
     * Get the ids of all registered configurable subsystems.
     *
     * @return The set of ids for all registered subsystems.
     * @see ConfigLabel
     * @see SubSystem
     */
    @NotNull
    public Set<String> getSubsystemNames() {
        Set<String> subsystemNames = new HashSet<>(autonomousConfig.keySet());
        subsystemNames.addAll(teleopConfig.keySet());
        return subsystemNames;
    }

    /**
     * Clears the config.
     */
    public void clearConfig() {
        autonomousConfig.clear();
        teleopConfig.clear();
        subsystemIdLookup.clear();
    }

    @NotNull
    @Contract(" -> new")
    @Override
    public HALConfig clone() {
        return new HALConfig(this);
    }
}