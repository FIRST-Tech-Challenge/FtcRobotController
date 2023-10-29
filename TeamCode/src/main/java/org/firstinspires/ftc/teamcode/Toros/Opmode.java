package org.firstinspires.ftc.teamcode.Toros;
//Package means where the class is stored
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//to import is to be able to use methods and classes of modules

//main class Opmode
@Disabled
public abstract class Opmode {

    public abstract void init();

    public abstract void loop();

    public class OpMode extends LinearOpMode {
        /// Decleration of DC motors switch them to the names of what is in you config
        //Same for servos and any other device connected to driver hub
        private DcMotor Motor1;
        private DcMotor Motor2;
        private DcMotor Motor3;
        private DcMotor Motor4;

        // Variable for moving the motors corresponding to axises on gamepad sticks
        //Speed
        double vertical = -gamepad1.left_stick_y;
        double pivot = gamepad1.left_stick_x * 1.1;
        double horizontal = gamepad1.right_stick_x;
        //denominator here is a list containing the absolute value of each direction as a maximum value for the motors to go when calculating the power
        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot),1);



        @Override
        public void runOpMode() throws InterruptedException {
            Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
            Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
            Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
            Motor4 = hardwareMap.get(DcMotor.class, "Motor4");

            waitForStart();
            if(opModeIsActive()){
                while (opModeIsActive()){
                    // sets motor power
                    Motor1.setPower((vertical + horizontal + pivot)/denominator);
                    Motor2.setPower((vertical - horizontal - pivot)/denominator);
                    Motor3.setPower((vertical - horizontal + pivot)/denominator);
                    Motor4.setPower((vertical + horizontal -  pivot)/denominator);
                }
            }
        }
    }

}
