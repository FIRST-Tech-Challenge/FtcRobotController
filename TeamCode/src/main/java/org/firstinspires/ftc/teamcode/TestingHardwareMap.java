package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */
public class TestingHardwareMap {

    /*
        | FRW               | Front Right Wheel     | Expansion Hub Motor 0   |
        | FLW               | Front Left Wheel      | Control Hub Motor 0     |
        | BRW               | Back Right Wheel      | Expansion Hub Motor 1   |
        | BLW               | Back Left Wheel       | Expansion Hub Motor 1   |
        | RIGHT_ODOMETER    | Right Odometer        | Expansion Hub Motor 3   |
        | LEFT_ODOMETER     | Left Odometer         | Control Hub Motor 3     |
        | CENTRE_ODOMETER   | Centre Odometer       | Expansion Hub Motor 2   |
     */

    /**
     * Get all DcMotors associated with driving
     * @return Array of DcMotors, in order of FrontRight | FrontLeft | BackRight | BackLeft
     */
    public static DcMotor[] GetDriveMotors(HardwareMap hardwareMap) {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");

        return new DcMotor[] {frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor};

    }
}
