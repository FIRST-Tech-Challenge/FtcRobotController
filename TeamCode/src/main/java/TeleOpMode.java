import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//A VERY VERY BASIC TANK DRIVE WITH JUST 2 MOTORS
//WE WILL CHANGE TO MECANUM LATER
//NOT ARCADE-DRIVE

//   ^------^
//  /  ^  ^  \
//  |   o    |
//  \_______/

@TeleOp(name="IshaanTeleOp")

public class TeleOpMode extends OpMode {
    //Register devices
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

    }
//VisAI?!?!?!?!
    //ViperSlide?!?!?
    @Override
    public void loop() {
        telemetry.addData("Ishaan_is", "COOL");
        telemetry.addData("LS-Y", gamepad1.left_stick_y);
        telemetry.addData("RS-Y", gamepad1.right_stick_y);

        fl.setPower(gamepad1.left_stick_y);
        fr.setPower(gamepad1.right_stick_y);
        bl.setPower(gamepad1.left_stick_y);
        br.setPower(gamepad1.right_stick_y);

        telemetry.addData("Ishaan_is", "COOL");
        telemetry.addData("LS-Y", gamepad1.left_stick_y);
        telemetry.addData("RS-Y", gamepad1.right_stick_y);

        telemetry.update();
        //I needed this apparently
    }
}

//LIBRARIES?!?!?!?!?!?!?