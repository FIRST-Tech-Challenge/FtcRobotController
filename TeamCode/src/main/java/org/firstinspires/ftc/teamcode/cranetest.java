package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous
public class cranetest extends LinearOpMode {


    private DcMotor crane;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Rev2mDistanceSensor distance2;
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");                            //mapping motors from control hub
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        crane = hardwareMap.get(DcMotor.class, "Crane");
        distance2=hardwareMap.get(Rev2mDistanceSensor.class,"distance 2");
        crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);                 //setting direction of drive train
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        if (opModeIsActive()) {
            /*crane.setTargetPosition(2000);
            crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            crane.setPower(1);
            while(crane.isBusy()&&opModeIsActive()){
                telemetry.addData("encoder value", crane.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("encoder value", crane.getCurrentPosition());
            telemetry.update();*/
            movewithdistacne(.5,12);
        }
    }
    public void movewithdistacne(double power,double distances) {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(distance2.getDistance(DistanceUnit.INCH)>=distances && opModeIsActive()){
            //frontRight.setPower(-power);
            //frontLeft.setPower(power);
            //backRight.setPower(power);
            //backLeft.setPower(-power-.2);
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}


