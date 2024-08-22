package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        static public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        static public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        static public double inPerTick = 1;
        static public double lateralInPerTick = inPerTick;
        static public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        static public double kS = 0;
        static public double kV = 0;
        static public double kA = 0;

        // path profile parameters (in inches)
        static public double maxWheelVel = 50;
        static public double minProfileAccel = -30;
        static public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        static public double maxAngVel = Math.PI; // shared with path
        static public double maxAngAccel = Math.PI;

        // path controller gains
        static public double axialGain = 0.0;
        static public double lateralGain = 0.0;
        static public double headingGain = 0.0; // shared with turn

        static public double axialVelGain = 0.0;
        static public double lateralVelGain = 0.0;
        static public double headingVelGain = 0.0; // shared with turn
    }