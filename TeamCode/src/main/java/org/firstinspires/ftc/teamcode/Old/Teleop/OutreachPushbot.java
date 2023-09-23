package org.firstinspires.ftc.teamcode.Old.Teleop;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "OutreachPushbot")
public class OutreachPushbot extends LinearOpMode{

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException{

    leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
    rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
    leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();


        while (opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("leftMotor", leftMotor.getCurrentPosition());
            telemetry.addData("rightMotor", rightMotor.getCurrentPosition());
            telemetry.update();


            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftMotor.setPower(drive+turn);
            rightMotor.setPower(drive-turn);


            }
    }

}
