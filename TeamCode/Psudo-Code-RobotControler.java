//If left stick forward, move all four motors move at a speed equa; to a factor of the place the left stick is at.
//If the left stick is moved backward, the 4 servos will move at a speed backward equal to a factor of the place the left stick is at.
//If the left stick is moved to the left the Frontleft and Backleft will rotate torwards each other and the Backright and Backleft will rotate away from each other
//If the left stick is moved to the right, the frontRight and backRight will rotate towards each other and the frontLeft and frontRight will rotate away from each other.
//If the Right stick is moved to the left, the frontLeft and backLeft will move to the back and the frontRight and backLeft will move forward at a rate equal to the force applied to the stick
//If the Right stick is moved to the right, the frontLeft and backLeft will move forward and the frontRight and backLeft will move in reverse at a rate equal to the force applied to the stick
// If the right stick is pushed down, the robot wil crouch until it is released, unless the mysterio medallion is equiped, therefore making it invisible.


package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotController extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
    }

    @Override
    public void loop() {
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;  
        double rightStickX = gamepad1.right_stick_x; 
        double rightStickY = gamepad1.right_stick_y; 

        
        if (leftStickY != 0) {
            frontLeft.setPower(leftStickY);
            frontRight.setPower(leftStickY);
            backLeft.setPower(leftStickY);
            backRight.setPower(leftStickY);
        }

        
        if (leftStickX != 0) {
            frontLeft.setPower(leftStickX);
            frontRight.setPower(-leftStickX);
            backLeft.setPower(-leftStickX);
            backRight.setPower(leftStickX);
        }

        
        if (rightStickX != 0) {
            frontLeft.setPower(rightStickX);
            frontRight.setPower(-rightStickX);
            backLeft.setPower(rightStickX);
            backRight.setPower(-rightStickX);
        }
      
}

