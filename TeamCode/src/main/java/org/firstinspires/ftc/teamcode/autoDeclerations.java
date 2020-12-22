package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class autoDeclerations {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFoward = null;
    private DcMotor rightReverse = null;
    private DcMotor leftReverse = null;
    private DcMotor rightFoward = null;
    private DcMotor intake = null;

    //Constructor
    public autoDeclerations() {}

    public void init(HardwareMap hardwareMap) {
            telemetry.addData("Status", "Initialized");

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            leftFoward  = hardwareMap.get(DcMotor.class, "left_foward_drive");
            rightReverse = hardwareMap.get(DcMotor.class, "right_reverse_drive");
            leftReverse = hardwareMap.get(DcMotor.class, "left_reverse_drive");
            rightFoward = hardwareMap.get(DcMotor.class, "right_foward_drive");
            intake = hardwareMap.get(DcMotor.class, "intake_intial");
            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftFoward.setDirection(DcMotor.Direction.FORWARD);
            rightReverse.setDirection(DcMotor.Direction.REVERSE);
            leftReverse.setDirection(DcMotor.Direction.FORWARD);
            rightFoward.setDirection(DcMotor.Direction.REVERSE);
            intake.setDirection(DcMotor.Direction.FORWARD);

        leftFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFoward.setPower(0);
        rightReverse.setPower(0);
        leftReverse.setPower(0);
        rightFoward.setPower(0);
        intake.setPower(0);



            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

    }
}
