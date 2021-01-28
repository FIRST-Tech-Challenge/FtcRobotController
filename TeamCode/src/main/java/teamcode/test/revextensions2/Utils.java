/*
 * Copyright (c) 2018 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.test.revextensions2;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * This class does Blackmagic
 */

class Utils
{
    static OpModeManagerImpl getOpModeManager()
    {
        return OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity());
    }

    static HardwareMap getHardwareMap()
    {
        return getOpModeManager().getHardwareMap();
    }

    static void hotswapHardwareMap()
    {
        HardwareMap map = getHardwareMap();

        swapStandardMotorsForLynx(map);
        swapStandardServosForLynx(map);
        addExpansionHubExForEachLynxModule(map);
    }

    static void deswapHardwareMap()
    {
        HardwareMap map = getHardwareMap();

        deswapLynxMotors(map);
        deswapLynxServos(map);
        removeExpansionHubExForEachLynxModule(map);
    }

    //------------------------------------------------------------------------------------------------------------------------------------------
    // Swapping
    //-------------------------------------------------------------------------------------------------------------------------------------------

    private static void swapStandardMotorsForLynx(HardwareMap map)
    {
        //-----------------------------------------------------------------------------------
        // Motors: standard --> Lynx
        //-----------------------------------------------------------------------------------

        ArrayList<Map.Entry<String, DcMotor>> motorsToRecreateAsLynx = new ArrayList<>();

        // TODO: Process both map.dcMotor and the root map.
        for (Map.Entry<String, DcMotor> entry : map.dcMotor.entrySet())
        {
            if (!(entry.getValue() instanceof ExpansionHubMotor)
                    && entry.getValue() instanceof DcMotorEx
                    && entry.getValue().getController() instanceof LynxDcMotorController)
            {
                motorsToRecreateAsLynx.add(entry);
            }
        }

        if (!motorsToRecreateAsLynx.isEmpty())
        {
            for (Map.Entry<String, DcMotor> entry : motorsToRecreateAsLynx)
            {
                map.dcMotor.remove(entry.getKey());
            }

            for (Map.Entry<String, DcMotor> entry : motorsToRecreateAsLynx)
            {
                map.dcMotor.put(entry.getKey(), new ExpansionHubMotor(entry.getValue()));
            }
        }
    }

    private static void swapStandardServosForLynx(HardwareMap map)
    {
        //-----------------------------------------------------------------------------------
        // Servos: standard --> Lynx
        //-----------------------------------------------------------------------------------

        ArrayList<Map.Entry<String, Servo>> servosToRecreateAsLynx = new ArrayList<>();

        // TODO: Process both map.servo and the root map.
        for (Map.Entry<String, Servo> entry : map.servo.entrySet())
        {
            if (!(entry.getValue() instanceof ExpansionHubServo)
                    && entry.getValue() instanceof ServoImplEx
                    && entry.getValue().getController() instanceof LynxServoController)
            {
                servosToRecreateAsLynx.add(entry);
            }
        }

        if (!servosToRecreateAsLynx.isEmpty())
        {
            for (Map.Entry<String, Servo> entry : servosToRecreateAsLynx)
            {
                map.servo.remove(entry.getKey());
            }

            for (Map.Entry<String, Servo> entry : servosToRecreateAsLynx)
            {
                map.servo.put(entry.getKey(), new ExpansionHubServo(entry.getValue()));
            }
        }
    }

    private static void addExpansionHubExForEachLynxModule(HardwareMap map)
    {
        //-----------------------------------------------------------------------------------
        // LynxModules --+> ExpansionHubEx
        //-----------------------------------------------------------------------------------

        HashMap<String, ExpansionHubEx> enhancedLynxModulesToInject = new HashMap<>();

        for (LynxModule module : map.getAll(LynxModule.class))
        {
            if (!hwMapContainsEnhancedModule(module))
            {
                enhancedLynxModulesToInject.put(getHwMapName(module), new ExpansionHubEx(module));
            }
        }

        for (Map.Entry<String, ExpansionHubEx> entry : enhancedLynxModulesToInject.entrySet())
        {
            map.put(entry.getKey(), entry.getValue());
        }
    }

    //------------------------------------------------------------------------------------------------------------------------------------------
    // Deswapping
    //-------------------------------------------------------------------------------------------------------------------------------------------

    private static void deswapLynxMotors(HardwareMap map)
    {
        //-----------------------------------------------------------------------------------
        // Motors: Lynx --> standard
        //-----------------------------------------------------------------------------------

        ArrayList<Map.Entry<String, DcMotor>> lynxMotorsToRecreateAsStandard = new ArrayList<>();

        for (Map.Entry<String, DcMotor> entry : map.dcMotor.entrySet())
        {
            if (entry.getValue() instanceof ExpansionHubMotor)
            {
                lynxMotorsToRecreateAsStandard.add(entry);
            }
        }

        if (!lynxMotorsToRecreateAsStandard.isEmpty())
        {
            for (Map.Entry<String, DcMotor> entry : lynxMotorsToRecreateAsStandard)
            {
                map.dcMotor.remove(entry.getKey());
            }

            for (Map.Entry<String, DcMotor> entry : lynxMotorsToRecreateAsStandard)
            {
                map.dcMotor.put(entry.getKey(),
                        new DcMotorImplEx(
                                entry.getValue().getController(),
                                entry.getValue().getPortNumber(),
                                entry.getValue().getDirection(),
                                entry.getValue().getMotorType()));
            }
        }

    }

    private static void deswapLynxServos(HardwareMap map)
    {
        //-----------------------------------------------------------------------------------
        // Servos: Lynx --> standard
        //-----------------------------------------------------------------------------------

        ArrayList<Map.Entry<String, Servo>> lynxServosToRecreateAsStandard = new ArrayList<>();

        for (Map.Entry<String, Servo> entry : map.servo.entrySet())
        {
            if (entry.getValue() instanceof ExpansionHubServo)
            {
                lynxServosToRecreateAsStandard.add(entry);
            }
        }

        if (!lynxServosToRecreateAsStandard.isEmpty())
        {
            for (Map.Entry<String, Servo> entry : lynxServosToRecreateAsStandard)
            {
                map.servo.remove(entry.getKey());
            }

            for (Map.Entry<String, Servo> entry : lynxServosToRecreateAsStandard)
            {
                map.servo.put(entry.getKey(),
                        new ServoImplEx(
                                (ServoControllerEx) entry.getValue().getController(),
                                entry.getValue().getPortNumber(),
                                entry.getValue().getDirection(),
                                ServoConfigurationType.getStandardServoType()));
            }
        }
    }

    private static void removeExpansionHubExForEachLynxModule(HardwareMap map)
    {
        //-----------------------------------------------------------------------------------
        // Remove ExpHbExs
        //-----------------------------------------------------------------------------------

        HashMap<String, ExpansionHubEx> enhancedLynxModulesToRemove = new HashMap<>();

        for (ExpansionHubEx module : map.getAll(ExpansionHubEx.class))
        {
            enhancedLynxModulesToRemove.put(getHwMapName(module), module);
        }

        if (!enhancedLynxModulesToRemove.isEmpty())
        {
            for (Map.Entry<String, ExpansionHubEx> entry : enhancedLynxModulesToRemove.entrySet())
            {
                map.remove(entry.getKey(), entry.getValue());
            }
        }
    }

    //------------------------------------------------------------------------------------------------------------------------------------------
    // Misc.
    //-------------------------------------------------------------------------------------------------------------------------------------------

    static String getHwMapName(HardwareDevice device)
    {
        return getHardwareMap().getNamesOf(device).iterator().next();
    }

    private static boolean hwMapContainsEnhancedModule(LynxModule module)
    {
        for(ExpansionHubEx enhancedModule : getHardwareMap().getAll(ExpansionHubEx.class))
        {
            //TODO: can we just replace with an '=='?
            if(module.getModuleAddress() == enhancedModule.getStandardModule().getModuleAddress())
            {
                return true;
            }
        }

        return false;
    }

    static LynxModule getLynxFromController(LynxController controller)
    {
        /*
         * NOTE: Historically we invoked the getModule() method
         * rather than grabbing the field directly. However, this
         * was reported to be causing problems in the wild because
         * getModule() actually returns a "PretendLynxModule" if
         * its internal "isHooked" boolean is false (as can happen
         * during a disconnect). This caused the cast to LynxModule
         * to fail and crashed the user code with an RE2Exception.
         *
         * See https://github.com/OpenFTC/RevExtensions2/issues/6
         */

        // We can get the underlying LynxModule object through a
        // LynxController object, but only through reflection.
        Field moduleField;

        try
        {
            // The "module" field is located within the LynxController class
            moduleField = LynxController.class.getDeclaredField("module");

            // Ensures the field is accessible for the next line. We still catch
            // the (impossible) IllegalAccessException just to be safe.
            moduleField.setAccessible(true);

            // Actually get the value from the controller that was passed in.
            return (LynxModule) moduleField.get(controller);
        }
        catch (Exception e)
        {
            throw new RE2Exception("Failed to reflect on LynxController! Please report this as an issue on the GitHub repository.");
        }
    }
}
