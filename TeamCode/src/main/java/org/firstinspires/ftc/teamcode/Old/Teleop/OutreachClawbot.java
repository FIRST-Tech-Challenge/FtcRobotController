package org.firstinspires.ftc.teamcode.Old.Teleop;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class OutreachClawbot extends LinearOpMode {
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private Servo claw = null;


    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.get(DcMotor.class, "motorRightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "motorLeftFront");
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");
        claw = hardwareMap.get(Servo.class, "claw");


        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();


        while(!isStopRequested()&&opModeIsActive()){
            telemetry.update();
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double strafe = gamepad1.right_stick_x;
            boolean openclaw = gamepad1.x;
            boolean closeclaw = gamepad1.y;
            telemetry.addData("strafe", Double.toString(strafe));
            telemetry.addData("drive", Double.toString(drive));
            telemetry.addData("turn", Double.toString(turn));

            frontLeft.setPower(drive+turn+strafe);
            backLeft.setPower(drive-turn+strafe);
            frontRight.setPower(drive-turn-strafe);
            backRight.setPower(drive+turn-strafe);
            if (openclaw == true){
                claw.setPosition(1);
            }
            if (openclaw == false){
                claw.setPosition(0);
            }




//            if (drive>0 || drive<0){
//                frontRight.setPower(drive);
//                frontLeft.setPower(drive);
//                backRight.setPower(drive);
//                backLeft.setPower(drive);
//            }
//
//            if (turn>0 || turn<0){
//                frontRight.setPower(turn);
//                backRight.setPower(turn);
//                frontLeft.setPower(-turn);
//                backLeft.setPower(-turn);
//            }
//
//            if (strafe>0 || strafe<0){
//                frontLeft.setPower(strafe);
//                backRight.setPower(strafe);
//                frontRight.setPower(-strafe);
//                backLeft.setPower(-strafe);
//            }



        }

    }
}
