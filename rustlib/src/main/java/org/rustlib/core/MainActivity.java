package org.rustlib.core;

import org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper;
import org.rustlib.rustboard.RustboardServer;

import java.lang.reflect.Field;

public class MainActivity extends PermissionValidatorWrapper {
    public MainActivity() {
        super();
        try {
            Field startApplication = PermissionValidatorWrapper.class.getDeclaredField("startApplication");
            startApplication.setAccessible(true);
            startApplication.set(null, RobotControllerActivity.class); // The first parameter can be null since this is a static field
        } catch (NoSuchFieldException | IllegalAccessException e) {
            RustboardServer.log(e);
            throw new RuntimeException("Unable to start the robot controller app.  Check that you're using the latest app version.");
        }
    }
}
