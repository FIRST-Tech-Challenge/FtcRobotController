package org.firstinspires.ftc.teamcode.alex;

// imports | do not touch
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

// VVV replace with @Autonomous for an autonomous program
@TeleOp //Declares the program to be a TeleOp
@Disabled

public class example extends LinearOpMode {

    //Private variables | These are directly tied to the DS configuration menu
    private Gyroscope imu;
    private DcMotor motorTest;
    private Servo servoTest;

    @Override
    public void runOpMode (){
        //get the references to the hardware devices that are listed in the Robot Controller’s configuration file
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest = hardwareMap.get(DcMotor.class, "motortest");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        //Telemetry sends data to the driver station, it can be very useful for debugging and testing
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        //everything written here occurs in the initialization phase (before the driver presses "start")

        while(opModeIsActive()){
            //everything written here will

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
