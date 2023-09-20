package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.CameraArray;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.Arrays;
import java.util.List;

// Robot class follows the singleton design pattern because it doesn't make since for there to be more than one instance
@SuppressWarnings("unused")
public enum Robot { SINGLETON;

    private static final List<? extends Subsystem> subsystems = Arrays.asList(Drivetrain.SINGLETON, CameraArray.SINGLETON);

    public static void initializeSubsystems(HardwareMap hardwareMap) // Static initialisation method
    {
        for(Subsystem system : subsystems)
        {
            system.initializeHardware(hardwareMap);
        }
    }
}
