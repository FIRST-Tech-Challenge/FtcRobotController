// File for centralising Robot hardware
package org.firstinspires.ftc.teamcode.core;

// Imports
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.hardware.Hardware;


// Class to centralise robot hardware
public class Robot {
    // Private variable for storing the hardware
    private final Hardware hardware;

    // Constructor for setting up the hardware
    Robot(HardwareMap hardwareMap) {
        if (hardwareMap == null) { throw new IllegalArgumentException("Hardware map cannot be null"); }
        this.hardware = new Hardware(hardwareMap);
    }
    // Method for returning the hardware
    public Hardware getHardware() { return this.hardware; }
}

// This file was created by: dev-verexxed
// Timestamp: [07/11/2026] [17:17]
