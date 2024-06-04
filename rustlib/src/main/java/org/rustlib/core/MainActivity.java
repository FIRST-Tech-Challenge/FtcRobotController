package org.rustlib.core;

import org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper;
import org.rustlib.config.Loader;
import org.rustlib.rustboard.Rustboard;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;

public class MainActivity extends PermissionValidatorWrapper {
    public MainActivity() {
        super();
        if (true) throw new RuntimeException("Hello from the RobotControllerActivity class");
        try {
            Loader.writeString(new File(Loader.defaultStorageDirectory, "log.txt"), "the main activity class has run");
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            Field startApplication = PermissionValidatorWrapper.class.getDeclaredField("startApplication");
            startApplication.setAccessible(true);
            startApplication.set(null, RobotControllerActivity.class); // The first parameter can be null since this is a static field
        } catch (NoSuchFieldException | IllegalAccessException e) {
            Rustboard.log(e);
            throw new RuntimeException("Unable to start the robot controller app.  Check that you're using the latest app version.");
        }
    }
}
