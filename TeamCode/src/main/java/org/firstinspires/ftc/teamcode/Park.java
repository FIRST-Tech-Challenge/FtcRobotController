package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "PowerPlayAuto", group = "pp")
public class Park extends LinearOpMode {
    Ppbot robot = new Ppbot();
    final double speedScalar = 0.8;
    //private ElapsedTime  runtime = new ElapsedTime();

    /*public DcMotor BLeft = null;
    public DcMotor BRight = null;
    public DcMotor FLeft = null;
    public DcMotor FRight = null;
    public DcMotor Slider = null;
    public Servo Take1 = null;
    public Servo Take2 = null;

    HardwareMap map = null;
    public void init(HardwareMap maps) {
        map = maps;
        BLeft = maps.dcMotor.get("bl");
        BRight = maps.dcMotor.get("br");
        FLeft = maps.dcMotor.get("fl");
        FRight = maps.dcMotor.get("fr");
        Take1 = maps.servo.get("grabber");
        //Take2 = maps.servo.get("grabber2");
        Slider = maps.dcMotor.get("slider");

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Slider.setDirection(DcMotorSimple.Direction.FORWARD);

        BLeft.setPower(0.0);
        BRight.setPower(0.0);
        FLeft.setPower(0.0);
        FRight.setPower(0.0);
        Slider.setPower(0.0);
        //Take1.setPosition(0.0); // change these 2 later when we figure out the servo positions
        //Take2.setPosition(0.0);

        BLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }*/

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello");
        telemetry.update();

        waitForStart();

        robot.BLeft.setPower(speedScalar * -0.7);
        robot.BRight.setPower(speedScalar * 0.7);
        robot.FRight.setPower(speedScalar * 1);
        robot.FLeft.setPower(speedScalar * 0.85);

        sleep(1000);

        robot.Take1.setPosition(0.0);
        robot.BLeft.setPower(0);
        robot.BRight.setPower(0);
        robot.FRight.setPower(0);
        robot.FLeft.setPower(0);
    }
}