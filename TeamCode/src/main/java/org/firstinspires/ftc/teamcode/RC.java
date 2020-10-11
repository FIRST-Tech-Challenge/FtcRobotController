package org.firstinspires.ftc.teamcode;

import android.app.Application;
import android.content.Context;
import android.view.View;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;


import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

/**
 * Created by tycho on 2/18/2017. This was copied from Team FIXIT. All hail team FIXIT.
 * This is simply a set of utils to reference the main robot controller activity for convenience
 */

public class RC {
    public static OpMode o;
    public static LinearOpMode l;

    public static HardwareMap h;
    public static int runNum = 0;
    //this is our old way of storing and accessing our vuforia key
    //maintained for compatibility with our older robot code
    //new method as of skystone 5.2 is to put the key in an xml file in res
    public final static String VUFORIA_LICENSE_KEY = "AbVe8An/////AAABmZuL3zqsJUfStpV5IU4Dp/p9KdvUSgvz7JuXGXwrFA4YEeDyH5BU3fbsp1mUKYLhA1WPX5r5E2nqv3sSkiP48oSuQRwWf7RTq7AfwxCY7qvldTj0ilT/XPb46/zyjbdZ7x/cQknV6zxt+rGLOiwRXID4wY/Tey52VMMoq1oxCFwogAXIWxZeF6DjmmfENbY6BwsXrAsIEHY3BQsdzI3HanDT6XJ+LUoPREvzi9Vh2iRhWiMX0E0pyWfs/El8qGl9tsQIEjaXp2Nax9zCKP8ehvr+8bwIF38qx+Rcmo1c9DH60fGFFzd4HW73UINTXwZvoJwCyh6KvBriLfDP8hcBXvStnd0JMi633BWsX5uZ+UiR";


    public static void setOpMode(OpMode op) {
        o = op;
        h = op.hardwareMap;


        if (op instanceof LinearOpMode) {
            l = (LinearOpMode) op;
        }

    }

    public static Context c() {
        return AppUtil.getInstance().getActivity();
    }//context
    public static FtcRobotControllerActivity a() {
        return ((FtcRobotControllerActivity) AppUtil.getInstance().getActivity());
    }
}