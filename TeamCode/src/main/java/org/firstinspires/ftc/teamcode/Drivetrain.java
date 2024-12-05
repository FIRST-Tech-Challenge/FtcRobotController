package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivetrain {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    public void init(HardwareMap hardwareMap){
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL.setDirection(FORWARD);
        motorFR.setDirection(FORWARD);
        motorBL.setDirection(FORWARD);
        motorBR.setDirection(FORWARD);

    }
    public void loop(double forwardBack, double strafe, double rotation){
        
        motorFL.setPower(forwardBack + strafe + rotation);
        motorFR.setPower(forwardBack - strafe - rotation);
        motorBL.setPower(forwardBack + strafe - rotation);
        motorBR.setPower(forwardBack - strafe + rotation);

    }

}
//Lefty = forwardback
//leftx = rotation
//rightx = strafe
