package org.firstinspires.ftc.teamcode.IntoTheDeep24_25.teleop;
import static com.qualcomm.robotcore.util.Range.clip;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TemplateJanx;

@TeleOp(name = "teleop arm code")
public class teleop_arm_code extends LinearOpMode {
    //replace Template with your class name
    //initiate motors
    // private motorType nameOfMotor
    private DcMotorEx turn;
    private Servo claw;
    @Override
    public void runOpMode() {
        TemplateJanx janx = new TemplateJanx(hardwareMap);
        janx.basicArmInit("arm","claw");
        turn = janx.motor;
        claw = janx.ser;
        //link motors to config
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //call functions here
                telemetry.addData("LSY:",gamepad2.left_stick_y);
                telemetry.update();
                arm(gamepad2.left_stick_y);
                servo();
            }
        }
    }

    //write functions here
    private void arm(double LSY){
        if(LSY < 0) {
         turn.setVelocity(-.5);
        }
        else if(LSY> 0){
            turn.setPower(.5);
        }
        else{
            turn.setPower(0);
        }
    }
    private void servo(){
        if(gamepad2.a)
        {
            claw.setPosition(1);
        }
        if(gamepad2.b)
        {
            claw.setPosition(-1);
        }
    }
}