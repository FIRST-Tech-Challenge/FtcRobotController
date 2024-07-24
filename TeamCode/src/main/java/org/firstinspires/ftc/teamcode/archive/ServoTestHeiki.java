
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="HeikiTest")
public class ServoTestHeiki extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    private Servo pull2 = null;
    private Servo pull1 = null;

    private Servo push1 = null;
    private Servo push2 = null;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        pull1  = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
        pull2 = hardwareMap.get(Servo.class, "Servo_Port_1_CH");
        push1  = hardwareMap.get(Servo.class, "Servo_Port_2_CH");
        push2 = hardwareMap.get(Servo.class, "Servo_Port_3_CH");
        
        double pull1Power = 0;
        double pull2Power = 0;
        double push1Power = 0;
        double push2Power = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            
            if(gamepad1.x){
                pull1Power=gamepad1.left_stick_y;
            }
            if(gamepad1.y){
                pull2Power=gamepad1.right_stick_y;
            }
            
            if(gamepad2.x){
                push1Power=gamepad1.left_stick_y;
            }
            if(gamepad2.y){
                push2Power=gamepad1.right_stick_y;
            }
            

            // Send calculated power to wheels
            pull1.setPosition(pull1Power/2+0.5);
            pull2.setPosition(pull2Power/2+0.5);
            push1.setPosition(push1Power/2+0.5);
            push2.setPosition(push2Power/2+0.5);

            // Show the elapsed game time and wheel power.
            telemetry.addData("1", pull1Power);
            telemetry.addData("2", pull2Power);
            telemetry.addData("3", push1Power);
            telemetry.addData("4", push1Power);
            telemetry.update();
        }
    }
}
