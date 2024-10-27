/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoHardwareMapper {

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public static void MapToHardware(LinearOpMode opMode, HornetRobo Robo)    {
        opMode.telemetry.addData("Hardware Mapper - MapToHardware:", "Starting to map");
        opMode.telemetry.update();

        //Mapping Wheel motors
        Robo.RightFrontMotor = opMode.hardwareMap.get(DcMotor.class, "motorRF");
        Robo.RightBackMotor = opMode.hardwareMap.get(DcMotor.class, "motorRB");
        Robo.LeftFrontMotor = opMode.hardwareMap.get(DcMotor.class, "motorLF");
        Robo.LeftBackMotor = opMode.hardwareMap.get(DcMotor.class, "motorLB");

        //Mapping Viper Slide Motors
        Robo.ViperSlideOne = opMode.hardwareMap.get(DcMotor.class, "motorvs");
        Robo.ViperSlideTwo = opMode.hardwareMap.get(DcMotor.class, "motorvstwo");

        //Mapping Arm Servos
        Robo.ArmOne = opMode.hardwareMap.get(Servo.class, "armServo");
        Robo.ArmTwo = opMode.hardwareMap.get(Servo.class, "armServoTwo");

        //Mapping Grabber Servos
        Robo.Grabber = opMode.hardwareMap.get(Servo.class, "gripperservo");

        opMode.telemetry.addData("Hardware Mapper - MapToHardware:", "Finished map");
        opMode.telemetry.update();
    }
}



