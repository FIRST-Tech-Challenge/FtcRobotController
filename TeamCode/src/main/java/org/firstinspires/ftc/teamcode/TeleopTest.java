package org.firstinspires.ftc.teamcode;
//Fix if detecting 2 or 0 minerals
//Give Power to Servo Motor holder
//Buttons to move latch and slide
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleopTest extends LinearOpMode{

    @Override
    public void runOpMode()  {
        Robot robot = new Robot (hardwareMap, telemetry);
        robot.isTeleOp = true;

        telemetry.addData ("Status", "Initialized"); //Displays "Status: Initialized"
        telemetry.update();
        //Driver must press INIT and then ▶️

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("TeleOP", "New Code");
            telemetry.update();

            /********** GamePad 1 ****************/
            //Turning
            if(this.gamepad1.left_stick_x < 0.5 && this.gamepad1.left_stick_x > -0.5){
                robot.turnOff();
            }
            if (this.gamepad1.left_stick_x > 0.5) {
                robot.turnForTime(0.8, 10, false, -1 );
            }

            if (this.gamepad1.left_stick_x < -0.5) {
                robot.turnForTime(0.8, 10, false, 1 );
            }

            // Moving
            if(this.gamepad1.right_stick_x < 0.5 && this.gamepad1.right_stick_x > -0.5 && this.gamepad1.right_stick_y < 0.5 && this.gamepad1.right_stick_y > -0.5){
                robot.turnOff();
            }
            if (this.gamepad1.right_stick_y > 0.5) {
                robot.moveF(1, 10);
            }

            if (this.gamepad1.right_stick_y < -0.5) {
                robot.moveB(1, 10);
            }

            if (this.gamepad1.right_stick_x > 0.5) {
                robot.moveR(1, 10);
            }

            if (this.gamepad1.right_stick_x < -0.5) {
                robot.moveL(1, 10);
            }

            /*if (this.gamepad1.left_trigger > 0.5) {
                robot.startIntake(10);
                robot.startShoot();
                robot.teleOpMotorBehavior();
            }

            if (this.gamepad1.b == true) {
                robot.startShoot();
            }
            if (this.gamepad1.b == false){
                robot.endShoot();
            }
            if (this.gamepad2.a == true){
                robot.openGrip();
            }
            if (this.gamepad2.b == true){
                robot.closeGrip();
            }
            if (this.gamepad2.dpad_up){robot.minRaise();}
            if (this.gamepad2.dpad_down){robot.minLower();}
            if(this.gamepad2.dpad_down == false && this.gamepad2.dpad_up == false){robot.stopWobble();}
            if(this.gamepad1.right_trigger > 0.5){
                robot.weakShot();
                robot.stopIntake();
            }*/
            if(this.gamepad1.dpad_left == false &&this.gamepad1.dpad_right == false && this.gamepad1.dpad_up == false && this.gamepad1.dpad_down == false){
                robot.turnOff();
            }

            if(this.gamepad1.dpad_left == true) {
                robot.moveR(0.5, 10);
            }

            if(this.gamepad1.dpad_right == true) {
                robot.moveL(0.5, 10);
            }

            if(this.gamepad1.dpad_up == true) {
                robot.moveF( 0.5,10);
            }

            if(this.gamepad1.dpad_down == true) {
                robot.moveB(0.5, 10);
            }


        };
    };
}
