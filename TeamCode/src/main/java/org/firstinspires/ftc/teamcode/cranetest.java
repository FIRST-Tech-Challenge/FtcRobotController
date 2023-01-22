package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class cranetest extends LinearOpMode {


    private DcMotor crane;

    public void runOpMode() {
        crane = hardwareMap.get(DcMotor.class, "Crane");
        crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        if (opModeIsActive()) {
            crane.setTargetPosition(2000);
            crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            crane.setPower(1);
            while(crane.isBusy()&&opModeIsActive()){
                telemetry.addData("encoder value", crane.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("encoder value", crane.getCurrentPosition());
            telemetry.update();
        }
    }
}


