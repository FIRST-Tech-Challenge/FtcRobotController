package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.Components.Switch.SwitchStates.PRESSED;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class Switch {

    DigitalChannel touchSensor;
    public enum SwitchStates {

        PRESSED(false, "PRESSED"),
        UNPRESSED(true, "UNPRESSED");

        boolean status;
        String name;

        SwitchStates(boolean p_status, String p_name) {
            this.status = p_status;
            this.name = p_name;
        }

        public void setStatus(boolean p_status) {
            this.status = p_status;
            if(p_status) {
                for (int i = 0; i < SwitchStates.values().length; i++) {
                    if (SwitchStates.values()[i] != this) {
                        SwitchStates.values()[i].status = false;
                    }
                }
            }
        }

        public boolean getStatus() {
            return this.status;
        }
    }
    public Switch() {
        touchSensor = op.hardwareMap.get(DigitalChannel.class, "clawSwitch");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
//        touchSensor.resetDeviceConfigurationForOpMode();
//        touchSensor.
//        touchSensor.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public boolean isSwitched() {
//        touchSensor.
        PRESSED.setStatus(touchSensor.getState());
        SwitchStates.UNPRESSED.setStatus(!touchSensor.getState());

        return PRESSED.getStatus();
    }

}
