/*
 * Copyright (c) 2024 DigitalChickenLabs
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Stack;

/*
 * This OpMode illustrates how to use the DigitalChickenLabs OctoQuad Quadrature Encoder & Pulse Width Interface Module.
 *
 *   The OctoQuad has 8 input channels that can used to read either Relative Quadrature Encoders or Pulse-Width Absolute Encoder inputs.
 *   Relative Quadrature encoders are found on most FTC motors, and some stand-alone position sensors like the REV Thru-Bore encoder.
 *   Pulse-Width encoders are less common. The REV Thru-Bore encoder can provide its absolute position via a variable pulse width,
 *   as can several sonar rangefinders such as the MaxBotix MB1000 series.
 *
 * This OpMode assumes that the OctoQuad is attached to an I2C interface named "octoquad" in the robot configuration.
 *
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * Select, Init and run the "OctoQuad Configuration Tool" OpMode
 * Read the blue User-Interface tips at the top of the telemetry screen.
 * Use the UI buttons to navigate the menu and make any desired changes to the OctoQuad configuration.
 * Use the Program Settings To FLASH option to make any changes permanent.
 *
 * See the sensor's product page: https://www.tindie.com/products/digitalchickenlabs/octoquad-8ch-quadrature-pulse-width-decoder/
 */
@TeleOp(name = "OctoQuad Configuration Tool", group="OctoQuad")
@Disabled
public class UtilityOctoQuadConfigMenu extends LinearOpMode
{
    TelemetryMenu.MenuElement rootMenu = new TelemetryMenu.MenuElement("OctoQuad Config Menu", true);
    TelemetryMenu.MenuElement menuHwInfo = new TelemetryMenu.MenuElement("Hardware Information", false);
    TelemetryMenu.EnumOption optionI2cResetMode;
    TelemetryMenu.EnumOption optionChannelBankConfig;

    TelemetryMenu.MenuElement menuEncoderDirections = new TelemetryMenu.MenuElement("Set Encoder Directions", false);
    TelemetryMenu.BooleanOption[] optionsEncoderDirections = new TelemetryMenu.BooleanOption[OctoQuad.NUM_ENCODERS];

    TelemetryMenu.MenuElement menuVelocityIntervals = new TelemetryMenu.MenuElement("Velocity Measurement Intervals", false);
    TelemetryMenu.IntegerOption[] optionsVelocityIntervals = new TelemetryMenu.IntegerOption[OctoQuad.NUM_ENCODERS];

    TelemetryMenu.MenuElement menuAbsParams = new TelemetryMenu.MenuElement("Abs. Encoder Pulse Width Params", false);
    TelemetryMenu.IntegerOption[] optionsAbsParamsMax = new TelemetryMenu.IntegerOption[OctoQuad.NUM_ENCODERS];
    TelemetryMenu.IntegerOption[] optionsAbsParamsMin = new TelemetryMenu.IntegerOption[OctoQuad.NUM_ENCODERS];

    TelemetryMenu.OptionElement optionProgramToFlash;
    TelemetryMenu.OptionElement optionSendToRAM;

    TelemetryMenu.StaticClickableOption optionExit;

    OctoQuad octoquad;

    boolean error = false;

    @Override
    public void runOpMode()
    {
        octoquad = hardwareMap.getAll(OctoQuad.class).get(0);

        if(octoquad.getChipId() != OctoQuad.OCTOQUAD_CHIP_ID)
        {
            telemetry.addLine("Error: cannot communicate with OctoQuad. Check your wiring and configuration and try again");
            telemetry.update();

            error = true;
        }
        else
        {
            if(octoquad.getFirmwareVersion().maj != OctoQuad.SUPPORTED_FW_VERSION_MAJ)
            {
                telemetry.addLine("Error: The OctoQuad is running a different major firmware version than this driver was built for. Cannot run configuration tool");
                telemetry.update();

                error = true;
            }
        }

        if(error)
        {
            waitForStart();
            return;
        }

        telemetry.addLine("Retrieving current configuration from OctoQuad");
        telemetry.update();

        optionExit = new TelemetryMenu.StaticClickableOption("Exit configuration menu")
        {
            @Override
            void onClick() // called on OpMode thread
            {
                requestOpModeStop();
            }
        };

        optionI2cResetMode = new TelemetryMenu.EnumOption("I2C Reset Mode", OctoQuad.I2cRecoveryMode.values(), octoquad.getI2cRecoveryMode());
        optionChannelBankConfig = new TelemetryMenu.EnumOption("Channel Bank Modes", OctoQuad.ChannelBankConfig.values(), octoquad.getChannelBankConfig());

        menuHwInfo.addChild(new TelemetryMenu.StaticItem("Board Firmware: v" + octoquad.getFirmwareVersion()));
        //menuHwInfo.addChild(new TelemetryMenu.StaticItem("Board unique ID: FIXME"));

        for(int i = 0; i < OctoQuad.NUM_ENCODERS; i++)
        {
            optionsEncoderDirections[i] = new TelemetryMenu.BooleanOption(
                    String.format("Encoder %d direction", i),
                    octoquad.getSingleEncoderDirection(i) == OctoQuad.EncoderDirection.REVERSE,
                    "-",
                    "+");
        }
        menuEncoderDirections.addChildren(optionsEncoderDirections);

        for(int i = 0; i < OctoQuad.NUM_ENCODERS; i++)
        {
            optionsVelocityIntervals[i] = new TelemetryMenu.IntegerOption(
                    String.format("Chan %d velocity intvl", i),
                    OctoQuad.MIN_VELOCITY_MEASUREMENT_INTERVAL_MS,
                    OctoQuad.MAX_VELOCITY_MEASUREMENT_INTERVAL_MS,
                    octoquad.getSingleVelocitySampleInterval(i));
        }
        menuVelocityIntervals.addChildren(optionsVelocityIntervals);

        for(int i = 0; i < OctoQuad.NUM_ENCODERS; i++)
        {
            OctoQuad.ChannelPulseWidthParams params = octoquad.getSingleChannelPulseWidthParams(i);

            optionsAbsParamsMax[i] = new TelemetryMenu.IntegerOption(
                    String.format("Chan %d max pulse length", i),
                    OctoQuad.MIN_PULSE_WIDTH_US,
                    OctoQuad.MAX_PULSE_WIDTH_US,
                    params.max_length_us);

            optionsAbsParamsMin[i] = new TelemetryMenu.IntegerOption(
                    String.format("Chan %d min pulse length", i),
                    OctoQuad.MIN_PULSE_WIDTH_US,
                    OctoQuad.MAX_PULSE_WIDTH_US,
                    params.min_length_us);
        }
        menuAbsParams.addChildren(optionsAbsParamsMin);
        menuAbsParams.addChildren(optionsAbsParamsMax);

        optionProgramToFlash = new TelemetryMenu.OptionElement()
        {
            String name = "Program Settings to FLASH";
            long lastClickTime = 0;

            @Override
            protected String getDisplayText()
            {
                if(lastClickTime == 0)
                {
                    return name;
                }
                else
                {
                    if(System.currentTimeMillis() - lastClickTime < 1000)
                    {
                        return name + " **OK**";
                    }
                    else
                    {
                        lastClickTime = 0;
                        return name;
                    }
                }
            }

            @Override
            void onClick()
            {
                sendSettingsToRam();
                octoquad.saveParametersToFlash();
                lastClickTime = System.currentTimeMillis();
            }
        };

        optionSendToRAM = new TelemetryMenu.OptionElement()
        {
            String name = "Send Settings to RAM";
            long lastClickTime = 0;

            @Override
            protected String getDisplayText()
            {
                if(lastClickTime == 0)
                {
                    return name;
                }
                else
                {
                    if(System.currentTimeMillis() - lastClickTime < 1000)
                    {
                        return name + " **OK**";
                    }
                    else
                    {
                        lastClickTime = 0;
                        return name;
                    }
                }
            }

            @Override
            void onClick()
            {
                sendSettingsToRam();
                lastClickTime = System.currentTimeMillis();
            }
        };

        rootMenu.addChild(menuHwInfo);
        rootMenu.addChild(optionI2cResetMode);
        rootMenu.addChild(optionChannelBankConfig);
        rootMenu.addChild(menuEncoderDirections);
        rootMenu.addChild(menuVelocityIntervals);
        rootMenu.addChild(menuAbsParams);
        rootMenu.addChild(optionProgramToFlash);
        rootMenu.addChild(optionSendToRAM);
        rootMenu.addChild(optionExit);

        TelemetryMenu menu = new TelemetryMenu(telemetry, rootMenu);

        while (!isStopRequested())
        {
            menu.loop(gamepad1);
            telemetry.update();
            sleep(20);
        }
    }

    void sendSettingsToRam()
    {
        for(int i = 0; i < OctoQuad.NUM_ENCODERS; i++)
        {
            octoquad.setSingleEncoderDirection(i, optionsEncoderDirections[i].getValue() ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
            octoquad.setSingleVelocitySampleInterval(i, optionsVelocityIntervals[i].getValue());

            OctoQuad.ChannelPulseWidthParams params = new OctoQuad.ChannelPulseWidthParams();
            params.max_length_us = optionsAbsParamsMax[i].getValue();
            params.min_length_us = optionsAbsParamsMin[i].getValue();

            octoquad.setSingleChannelPulseWidthParams(i, params);
        }

        octoquad.setI2cRecoveryMode((OctoQuad.I2cRecoveryMode) optionI2cResetMode.getValue());
        octoquad.setChannelBankConfig((OctoQuad.ChannelBankConfig) optionChannelBankConfig.getValue());
    }

    /*
     * Copyright (c) 2023 OpenFTC Team
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

    public static class TelemetryMenu
    {
        private final MenuElement root;
        private MenuElement currentLevel;

        private boolean dpadUpPrev;
        private boolean dpadDnPrev;
        private boolean dpadRightPrev;
        private boolean dpadLeftPrev;
        private boolean xPrev;
        private boolean lbPrev;

        private int selectedIdx = 0;
        private Stack<Integer> selectedIdxStack = new Stack<>();

        private final Telemetry telemetry;

        /**
         * TelemetryMenu constructor
         * @param telemetry pass in 'telemetry' from your OpMode
         * @param root the root menu element
         */
        public TelemetryMenu(Telemetry telemetry, MenuElement root)
        {
            this.root = root;
            this.currentLevel = root;
            this.telemetry = telemetry;

            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            telemetry.setMsTransmissionInterval(50);
        }

        /**
         * Call this from inside your loop to put the current menu state into
         * telemetry, and process gamepad inputs for navigating the menu
         * @param gamepad the gamepad you want to use to navigate the menu
         */
        public void loop(Gamepad gamepad)
        {
            // Capture current state of the gamepad buttons we care about;
            // We can only look once or we risk a race condition
            boolean dpadUp = gamepad.dpad_up;
            boolean dpadDn = gamepad.dpad_down;
            boolean dpadRight = gamepad.dpad_right;
            boolean dpadLeft = gamepad.dpad_left;
            boolean x = gamepad.x;
            boolean lb = gamepad.left_bumper;

            // Figure out who our children our at this level
            // and figure out which item is currently highlighted
            // with the selection pointer
            ArrayList<Element> children = currentLevel.children();
            Element currentSelection = children.get(selectedIdx);

            // Left and right are inputs to the selected item (if it's an Option)
            if (currentSelection instanceof OptionElement)
            {
                if (dpadRight && !dpadRightPrev) // rising edge
                {
                    ((OptionElement) currentSelection).onRightInput();
                }
                else if (dpadLeft && !dpadLeftPrev) // rising edge
                {
                    ((OptionElement) currentSelection).onLeftInput();
                }
            }

            // Up and down navigate the current selection pointer
            if (dpadUp && !dpadUpPrev) // rising edge
            {
                selectedIdx--; // Move selection pointer up
            }
            else if (dpadDn && !dpadDnPrev) // rising edge
            {
                selectedIdx++; // Move selection pointer down
            }

            // Make selected index sane (don't let it go out of bounds) :eyes:
            if (selectedIdx >= children.size())
            {
                selectedIdx = children.size()-1;
            }
            else if (selectedIdx < 0)
            {
                selectedIdx = 0;
            }

            // Select: either enter submenu or input to option
            else if (x && !xPrev) // rising edge
            {
                // Select up element
                if (currentSelection instanceof SpecialUpElement)
                {
                    // We can only go up if we're not at the root level
                    if (currentLevel != root)
                    {
                        // Restore selection pointer to where it was before
                        selectedIdx = selectedIdxStack.pop();

                        // Change to the parent level
                        currentLevel = currentLevel.parent();
                    }
                }
                // Input to option
                else if (currentSelection instanceof OptionElement)
                {
                    ((OptionElement) currentSelection).onClick();
                }
                // Enter submenu
                else if (currentSelection instanceof MenuElement)
                {
                    // Save our current selection pointer so we can restore it
                    // later if the user navigates back up a level
                    selectedIdxStack.push(selectedIdx);

                    // We have no idea what's in the submenu :monkey: so best to
                    // just set the selection pointer to the first element
                    selectedIdx = 0;

                    // Now the current level becomes the submenu that the selection
                    // pointer was on
                    currentLevel = (MenuElement) currentSelection;
                }
            }

            // Go up a level
            else if (lb && !lbPrev)
            {
                // We can only go up if we're not at the root level
                if (currentLevel != root)
                {
                    // Restore selection pointer to where it was before
                    selectedIdx = selectedIdxStack.pop();

                    // Change to the parent level
                    currentLevel = currentLevel.parent();
                }
            }

            // Save the current button states so that we can look for
            // the rising edge the next time around the loop :)
            dpadUpPrev = dpadUp;
            dpadDnPrev = dpadDn;
            dpadRightPrev = dpadRight;
            dpadLeftPrev = dpadLeft;
            xPrev = x;
            lbPrev = lb;

            // Start building the text display.
            // First, we add the static directions for gamepad operation
            StringBuilder builder = new StringBuilder();
            builder.append("<font color='#119af5' face=monospace>");
            builder.append("Navigate items.....dpad up/down\n")
                    .append("Select.............X\n")
                    .append("Edit option........dpad left/right\n")
                    .append("Up one level.......left bumper\n");
            builder.append("</font>");
            builder.append("\n");

            // Now actually add the menu options. We start by adding the name of the current menu level.
            builder.append("<font face=monospace>");
            builder.append("Current Menu: ").append(currentLevel.name).append("\n");

            // Now we loop through all the child elements of this level and add them
            for (int i = 0; i < children.size(); i++)
            {
                // If the selection pointer is at this index, put a green dot in the box :)
                if (selectedIdx == i)
                {
                    builder.append("[<font color=green face=monospace>•</font>] ");
                }
                // Otherwise, just put an empty box
                else
                {
                    builder.append("[ ] ");
                }

                // Figure out who the selection pointer is pointing at :eyes:
                Element e = children.get(i);

                // If it's pointing at a submenu, indicate that it's a submenu to the user
                // by prefixing "> " to the name.
                if (e instanceof MenuElement)
                {
                    builder.append("> ");
                }

                // Finally, add the element's name
                builder.append(e.getDisplayText());

                // We musn't forget the newline
                builder.append("\n");
            }

            // Don't forget to close the font tag either
            builder.append("</font>");

            // Build the string!!!! :nerd:
            String menu = builder.toString();

            // Add it to telemetry
            telemetry.addLine(menu);
        }

        public static class MenuElement extends Element
        {
            private String name;
            private ArrayList<Element> children = new ArrayList<>();

            /**
             * Create a new MenuElement; may either be the root menu, or a submenu (set isRoot accordingly)
             * @param name the name for this menu
             * @param isRoot whether this is a root menu, or a submenu
             */
            public MenuElement(String name, boolean isRoot)
            {
                this.name = name;

                // If it's not the root menu, we add the up one level option as the first element
                if (!isRoot)
                {
                    children.add(new SpecialUpElement());
                }
            }

            /**
             * Add a child element to this menu (may either be an Option or another menu)
             * @param child the child element to add
             */
            public void addChild(Element child)
            {
                child.setParent(this);
                children.add(child);
            }

            /**
             * Add multiple child elements to this menu (may either be option, or another menu)
             * @param children the children to add
             */
            public void addChildren(Element[] children)
            {
                for (Element e : children)
                {
                    e.setParent(this);
                    this.children.add(e);
                }
            }

            @Override
            protected String getDisplayText()
            {
                return name;
            }

            private ArrayList<Element> children()
            {
                return children;
            }
        }

        public static abstract class OptionElement extends Element
        {
            /**
             * Override this to get notified when the element is clicked
             */
            void onClick() {}

            /**
             * Override this to get notified when the element gets a "left edit" input
             */
            protected void onLeftInput() {}

            /**
             * Override this to get notified when the element gets a "right edit" input
             */
            protected void onRightInput() {}
        }

        public static class EnumOption extends OptionElement
        {
            protected int idx = 0;
            protected Enum[] e;
            protected String name;

            public EnumOption(String name, Enum[] e)
            {
                this.e = e;
                this.name = name;
            }

            public EnumOption(String name, Enum[] e, Enum def)
            {
                this(name, e);
                idx = def.ordinal();
            }

            @Override
            public void onLeftInput()
            {
                idx++;

                if(idx > e.length-1)
                {
                    idx = 0;
                }
            }

            @Override
            public void onRightInput()
            {
                idx--;

                if(idx < 0)
                {
                    idx = e.length-1;
                }
            }

            @Override
            public void onClick()
            {
                onRightInput();
            }

            @Override
            protected String getDisplayText()
            {
                return String.format("%s: <font color='#e37c07' face=monospace>%s</font>", name, e[idx].name());
            }

            public Enum getValue()
            {
                return e[idx];
            }
        }

        public static class IntegerOption extends OptionElement
        {
            protected int i;
            protected int min;
            protected int max;
            protected String name;

            public IntegerOption(String name, int min, int max, int def)
            {
                this.name = name;
                this.min = min;
                this.max = max;
                this.i = def;
            }

            @Override
            public void onLeftInput()
            {
                i--;

                if(i < min)
                {
                    i = max;
                }
            }

            @Override
            public void onRightInput()
            {
                i++;

                if(i > max)
                {
                    i = min;
                }
            }

            @Override
            public void onClick()
            {
                onRightInput();
            }

            @Override
            protected String getDisplayText()
            {
                return String.format("%s: <font color='#e37c07' face=monospace>%d</font>", name, i);
            }

            public int getValue()
            {
                return i;
            }
        }

        static class BooleanOption extends OptionElement
        {
            private String name;
            private boolean val = true;

            private String customTrue;
            private String customFalse;

            BooleanOption(String name, boolean def)
            {
                this.name = name;
                this.val = def;
            }

            BooleanOption(String name, boolean def, String customTrue, String customFalse)
            {
                this(name, def);
                this.customTrue = customTrue;
                this.customFalse = customFalse;
            }

            @Override
            public void onLeftInput()
            {
                val = !val;
            }

            @Override
            public void onRightInput()
            {
                val = !val;
            }

            @Override
            public void onClick()
            {
                val = !val;
            }

            @Override
            protected String getDisplayText()
            {
                String valStr;

                if(customTrue != null && customFalse != null)
                {
                    valStr = val ? customTrue : customFalse;
                }
                else
                {
                    valStr = val ? "true" : "false";
                }

                return String.format("%s: <font color='#e37c07' face=monospace>%s</font>", name, valStr);
            }

            public boolean getValue()
            {
                return val;
            }
        }

        /**
         *
         */
        public static class StaticItem extends OptionElement
        {
            private String name;

            public StaticItem(String name)
            {
                this.name = name;
            }

            @Override
            protected String getDisplayText()
            {
                return name;
            }
        }

        public static abstract class StaticClickableOption extends OptionElement
        {
            private String name;

            public StaticClickableOption(String name)
            {
                this.name = name;
            }

            abstract void onClick();

            @Override
            protected String getDisplayText()
            {
                return name;
            }
        }

        private static abstract class Element
        {
            private MenuElement parent;

            protected void setParent(MenuElement parent)
            {
                this.parent = parent;
            }

            protected MenuElement parent()
            {
                return parent;
            }

            protected abstract String getDisplayText();
        }

        private static class SpecialUpElement extends Element
        {
            @Override
            protected String getDisplayText()
            {
                return "<font color='#119af5' face=monospace>.. ↰ Up One Level</font>";
            }
        }
    }
}
