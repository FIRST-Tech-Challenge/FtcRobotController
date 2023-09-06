package org.firstinspires.ftc.teamcode.util;

public class Log {
    private static Log instance;
    private boolean command_line;
    private Log() {
        switch (System.getProperty("os.name").substring(0,3)) {
            case "Wind":
                break;
            default:
                break;
            //windows is the OS
        }
    }

    public static Log getInstance() {

        if (instance == null) {
            instance = new Log();
        }
        return instance;
    }


}
