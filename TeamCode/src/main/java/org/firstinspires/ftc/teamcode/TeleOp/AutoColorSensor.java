package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Functions.ColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

@Disabled
public class AutoColorSensor extends OpMode {

    ColorSensor colorSensor;
    RevColorSensorV3 REVV3;

    @Override
    public void init() {
        colorSensor = new ColorSensor(REVV3);

    }

    @Override
    public void loop() {

        boolean getWhite = colorSensor.checkForWhite();
        boolean getYellow = colorSensor.checkForYellow();


        if(getWhite == true)
        {
            telemetry.addLine("Incarcat: sfera alba!"); //daca identifica culoare alba arata un mesaj
        }
        else if(getYellow == true)
        {
            telemetry.addLine("Incarcat: cubul galben!"); //daca identifica culoare galbena arata un mesaj
        }
        else {
            telemetry.addLine("Descarcat!"); //daca nu identifica nici o culoare din cele de mai sus arata un mesaj
        }
        telemetry.update();

    }

}
