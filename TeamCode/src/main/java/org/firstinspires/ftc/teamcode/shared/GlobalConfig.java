package org.firstinspires.ftc.teamcode.shared;

public class GlobalConfig {


    public GlobalConfig() {

    }
    public GlobalConfig(AUTONOMOUS_DELIVERY_MODES activeDeliveryMode) {
        this.activeDeliveryMode = activeDeliveryMode;
    }

    public static enum ALLIANCE_POS {
        LEFT,
        RIGHT
    }

    public static enum ALLIANCE_COL {
        BLUE,
        RED
    }

    public static enum AUTONOMOUS_DELIVERY_MODES {
        ARM,
        DROPPER
    }

    private AUTONOMOUS_DELIVERY_MODES activeDeliveryMode;

    public AUTONOMOUS_DELIVERY_MODES getActiveDeliveryMode() {
        return activeDeliveryMode;
    }

    public void setActiveDeliveryMode(AUTONOMOUS_DELIVERY_MODES activeDeliveryMode) {
        this.activeDeliveryMode = activeDeliveryMode;
    }


}
