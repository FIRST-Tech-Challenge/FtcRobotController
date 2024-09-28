package org.firstinspires.ftc.team20150;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "2 Driver TeleOp", group = "Iterative OpMode")

public class TwoDriverTeleOpMode extends BaseTeleOpMode {

    @Override
    public void loop() {

        // Save CPU resources; can resume streaming when needed.
        if (gamepad2.dpad_down) {
            intakeMotor.setPower(1.0);
        } else if (gamepad2.dpad_up) {
            intakeMotor.setPower(-1.0); 
        }
        else{
            intakeMotor.setPower(0);
        }
        
        // arm control
        double armMaxStep = 4.0;
        double armStep = -gamepad2.left_stick_y * armMaxStep;
        armTargetPosition += (int)Math.round(armStep);

        if (armTargetPosition < 0)
            armTargetPosition = 0;
        if (armTargetPosition > 750)
            armTargetPosition = 750;

        armMotor.setTargetPosition(armTargetPosition);
        
        //box servo control
        //double minBoxPosition = 0.25;
        double minBoxPosition = 0.3;
        double maxBoxPosition = 0.86;
        
        //boxServo = hardwareMap.get(Servo.class, "BOX");

        int armPos = armMotor.getCurrentPosition();
        double armAngleDeg = armPos * 0.12;// * 0.21;

        //telemetry.addData("armAngle: ", "%4.2f", armAngleDeg);

        double boxPosition;

        if (armPos <= 400 && !gamepad2.square) {
            double boxTuck = (armPos < 15) ? 0.0 : armAngleDeg * 0.002;
            boxPosition = minBoxPosition - boxTuck;
        } else {
            boxPosition = (boxServo.getPosition() + gamepad2.right_stick_y * -0.035);
            if (boxPosition < minBoxPosition)
                boxPosition = minBoxPosition;
            else if (boxPosition > maxBoxPosition)
                boxPosition = maxBoxPosition;
        }

        /*
        if (armPos <= 300) {
            if (armPos < 20)
                boxPosition = 0.30;
            else if (armPos < 50)
                boxPosition = 0.20;
            else if (armPos < 160)
                boxPosition = 0.25;
            else if (armPos < 200) 
                boxPosition = 0.25;
            else
                boxPosition = 0.24;
        } else {
            if (boxPosition < minBoxPosition)
                boxPosition = minBoxPosition;
            else if (boxPosition > maxBoxPosition)
                boxPosition = maxBoxPosition;
        }
        */
        
        telemetry.addData("armPos:", "%d", armPos);
      
        boxServo.setPosition(boxPosition);

        //airplane launcher
        if(gamepad1.right_bumper) {
            //planeServo.setPosition(planeServoDropPosition);
        }
        else {
            //planeServo.setPosition(planeServoHoldPosition);
        }

        //purple pixel override
        if(gamepad2.left_bumper){
            purpleServo.setPosition(purpleServoDropPosition);
        }
        else{
            purpleServo.setPosition(purpleServoHoldPosition);
        }

        super.loop();
    }
}
