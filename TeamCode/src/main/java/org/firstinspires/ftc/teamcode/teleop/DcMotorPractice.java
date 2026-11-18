package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.TestBench;

@TeleOp
public class DcMotorPractice extends OpMode {

    private TestBench bench;
    private RobotHardwareMap robotHardwareMap;
    private DcMotorEx motor;

    @Override
    public void init(){
        robotHardwareMap = new RobotHardwareMap(hardwareMap,this);
        robotHardwareMap.initialize();
        bench = new TestBench(robotHardwareMap);
        bench.init(hardwareMap);
        this.motor = hardwareMap.get(DcMotorEx.class,"launcher_pollen");

//        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        bench.getLauncherNectar().setDirection(DcMotorSimple.Direction.REVERSE);
//        bench.getIntake().setDirection(DcMotorSimple.Direction.REVERSE);
//        bench.getTransfer().setDirection(DcMotorSimple.Direction.REVERSE);
//
    }
    double speedIncrement = 0;
    @Override
    public void loop(){

        telemetry.addData("Motor 3 Type",motor.getMotorType());
//        bench.setMotorSpeed(1500);
//        bench.setMotorSpeed(1500);

        telemetry.addData("TicksPerRev",bench.ticksPerRev);
        telemetry.addData("Motor Revs",bench.getMotorRev());
        telemetry.addData("a",gamepad1.a);
        telemetry.addData("b",gamepad1.b);
        telemetry.addData("y",gamepad1.y);
        telemetry.addData("power",speedIncrement);
        telemetry.addData("velocity",motor.getVelocity());
        telemetry.addData("rpm 19.2",motor.getVelocity()*60/28/19.2);
        telemetry.addData("rpm1:1 6000",motor.getVelocity()*60/28);
//        updateTelemetry("rpm",motor.);
        if (gamepad1.bWasPressed()){
            speedIncrement += .05;
        }
        if (gamepad1.yWasPressed()){
            speedIncrement -= .05;
        }
        if (gamepad1.a) {
           bench.setMotorSpeed(0);
           bench.getLauncherNectar().setPower(0);
            bench.getIntake().setPower(0);
            bench.getTransfer().setPower(0);
        } else {
            bench.setMotorSpeed(speedIncrement);
            bench.getLauncherNectar().setPower(speedIncrement);
            bench.getIntake().setPower(speedIncrement);
            bench.getTransfer().setPower(speedIncrement);


        }
        if (gamepad1.right_trigger > 0){
            bench.getServo().setPower(.7);
        } else {
            bench.getServo().setPower(0);
        }
    }
}
