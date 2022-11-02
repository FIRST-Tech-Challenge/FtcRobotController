package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.DataLogger.DataLoggerController;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;

@TeleOp(name="DATA LOGGER TEST", group="TEST")
@Disabled
// Facut de Vlad
public class DataLoggerControllerTest extends OpMode {


    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private Move move;
    private Rotate rotate;
    private DataLoggerController dataLoggerController;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        VoltageSensor VS =  this.hardwareMap.voltageSensor.iterator().next();


        dataLoggerController = new DataLoggerController("DataLoggerControllerTest", gamepad1, gamepad2, VS);
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
    }

    public void start(){
        runtime.reset();
        runtime.startTime();
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) //merge in fata
        {
            move.MoveFull(1);
            dataLoggerController.functions.gamepad1DpadUp = "merge in fata";
        }
        else if(gamepad1.dpad_down) //merge in spate
        {
            move.MoveFull(2);
            dataLoggerController.functions.gamepad1DpadDown = "merge in spate";
        }
        else if(gamepad1.dpad_left) //merge stanga
        {
            rotate.RotateFull(1);
            dataLoggerController.functions.gamepad1DpadLeft = "roteste in stanga";

        }
        else if(gamepad1.dpad_right) // merge dreapta
        {
            rotate.RotateFull(2);
            dataLoggerController.functions.gamepad1DpadRight = "roteste in dreapta";
        }
        //miscarea stanga-dreapta:
        else if(gamepad1.left_bumper)
        {
            move.MoveFull(3);
            dataLoggerController.functions.gamepad1DpadRight = "slide in stanga";
        }
        else if(gamepad1.right_bumper)
        {
            move.MoveFull(4);
            dataLoggerController.functions.gamepad1DpadRight = "slide in dreapta";
        }
        if(!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.right_bumper && !gamepad1.left_bumper
                && gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x==0 && !gamepad1.a && !gamepad1.b && gamepad1.right_stick_y==0)
        {
            move.MoveStop();
        }

        dataLoggerController.WriteData(runtime.time());
    }

    @Override
    public void stop(){
        dataLoggerController.WriteEnd(runtime.time());
    }
}
