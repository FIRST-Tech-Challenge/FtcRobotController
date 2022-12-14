package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.MV.MVGamepad;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name="custom", group="TEST")
@Disabled
public class CustomMain extends OpMode {

    List<String> buttons= new ArrayList<>();
    List<String> buttonsLabels= new ArrayList<>();

    MVGamepad gamepad;

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    public Move move;
    public Rotate rotate;

    @Override
    public void init() {

        // add button labels
        //leftMotor = hardwareMap.dcMotor.get("FL");
        //rightMotor = hardwareMap.dcMotor.get("FR");
        //leftMotorBack = hardwareMap.dcMotor.get("BL");
        //rightMotorBack = hardwareMap.dcMotor.get("BR");
        //move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        //rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);

        buttons= new ArrayList<>(0);
        buttonsLabels= new ArrayList<>(0);

        buttonsLabels.add("primul buton:");
        buttonsLabels.add("al doilea buton:");

        gamepad=new MVGamepad(gamepad1);
        /*
        for(int i=0;i<buttonsLabels.size();i++){
            while(gamepad.ReturnNamePressedButton()==""){
                TelemetryShow("Press to select "+buttonsLabels.get(i));
                sleep(100);
            }
            buttons.add(gamepad.ReturnNamePressedButton());
            TelemetryShow("Press to select "+buttonsLabels.get(i));
            TelemetryShow("Selected button: "+buttons.get(i));
            sleep(3000);
        }

         */
    }



    @Override
    public void init_loop() {
        if(buttons.size()<=buttonsLabels.size()) {
            if (buttons.size() - 1 >= 0) {
                TelemetryShow("Selected button: " + buttons.get(buttons.size() - 1));
            }
            TelemetryShow("Press to select " + buttonsLabels.get(buttons.size()));
            if (gamepad.ReturnNamePressedButton() != "") {
                buttons.add(gamepad.ReturnNamePressedButton());
            }
        }
        else{
            TelemetryShow("Done!");
            for (int i = 0; i<=buttons.size()-1 ; i++) {
                TelemetryShow(i+". "+buttonsLabels.get(i)+" "+buttons.get(i)+"/n");
            }
        }

    }

    @Override
    public void loop() {
        if(gamepad.CheckPressed(buttons.get(0))){
            TelemetryShow("Go up!");
        }
        else if(gamepad.CheckPressed(buttons.get(1))){
            TelemetryShow("Go down!");
        }
    }

    void TelemetryShow(String text){
        telemetry.addLine(text);
        telemetry.update();
    }

    public void sleep(int milis){
        try {
            Thread.sleep(milis);
            this.msStuckDetectInit=msStuckDetectInit+milis;
        } catch (Exception e){}
    }
}
