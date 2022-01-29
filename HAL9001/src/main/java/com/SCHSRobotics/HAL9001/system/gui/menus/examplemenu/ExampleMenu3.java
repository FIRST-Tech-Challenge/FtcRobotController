package com.SCHSRobotics.HAL9001.system.gui.menus.examplemenu;

import com.SCHSRobotics.HAL9001.system.gui.HALMenu;
import com.SCHSRobotics.HAL9001.system.gui.Payload;
import com.SCHSRobotics.HAL9001.system.gui.event.DataPacket;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.TextElement;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EntireViewButton;
import com.SCHSRobotics.HAL9001.util.control.Button;

public class ExampleMenu3 extends HALMenu {
    @Override
    protected void init(Payload payload) {
        addItem(new TextElement("Hey, you're not supposed to be here!"));
        addItem(new EntireViewButton()
                .onClick(new Button<>(1, Button.BooleanInputs.b), (DataPacket packet) -> {
                    gui.back();
                }));
    }
}
