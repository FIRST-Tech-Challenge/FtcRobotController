package com.SCHSRobotics.HAL9001.system.gui.menus.examplemenu;

import com.SCHSRobotics.HAL9001.system.gui.HALMenu;
import com.SCHSRobotics.HAL9001.system.gui.Payload;
import com.SCHSRobotics.HAL9001.system.gui.SelectionZone;
import com.SCHSRobotics.HAL9001.system.gui.event.DataPacket;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.TextElement;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.ViewButton;
import com.SCHSRobotics.HAL9001.util.control.Button;

public class ExampleMenu2 extends HALMenu {
    @Override
    protected void init(Payload payload) {
        selectionZone = new SelectionZone(2,2);
        addItem(new ViewButton("## | Sorry Mario, the princess is in another castle.")
                    .onClick(new Button<>(1, Button.BooleanInputs.y), (DataPacket packet) -> gui.back())
                    .onClick(new Button<>(1, Button.BooleanInputs.a), (DataPacket packet) -> gui.inflate(new ExampleMenu3())));
        addItem(new TextElement("#"));
    }
}
