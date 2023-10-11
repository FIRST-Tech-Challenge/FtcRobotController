package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
This is making the "Hardware" class public, so it can be used by other files.
 */

public class Hardware {

    /*
    These two lines are declaring the names and types of the objects.
     */

    public DcMotor  TestMotor = null;
    public Servo    TestServo = null;

    /*
    Creating hwMap as an object that can only be used in this class.
     */
    HardwareMap     hwMap = null;

    /*
    Technically not necessary, but creates Hardware as a public object to be used in another file.

    Java does this automatically with all objects that don't have a constructor, but it's good
    practice to include.
     */
    public Hardware (){
    }

    /*
    Creates a method to import the hardware on initialization of the program.
     */
    public void init(HardwareMap ahwMap){

        /*
        Saves a reference to the hwMap class made by FTC
         */
        hwMap = ahwMap;

        /*
        Defining what type of hardware "TestMotor" and "TestServo" are.
         */
        TestMotor = hwMap.get(DcMotor.class, "TestMotor");
        TestServo = hwMap.get(Servo.class, "TestServo");

    }
}
