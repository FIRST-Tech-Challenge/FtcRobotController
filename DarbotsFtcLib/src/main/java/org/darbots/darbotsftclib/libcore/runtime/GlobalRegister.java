package org.darbots.darbotsftclib.libcore.runtime;

import com.qualcomm.hardware.lynx.LynxModule;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.OpModeRunLog;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;

import java.util.List;

public class GlobalRegister {
    public static List<LynxModule> allExtensionHubs = null;
    public static DarbotsBasicOpMode runningOpMode = null;
    public static OpModeRunLog currentLog = null;
    public static RobotCore currentRobotCore = null;
}
