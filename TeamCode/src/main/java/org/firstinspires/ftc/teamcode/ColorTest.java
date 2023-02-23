package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorTest", group ="TeleOp")
public class ColorTest extends OpMode {

    public ColorSensor color;
    public DcMotor Slide;


    @Override
    public void init() {
        color = hardwareMap.get(ColorSensor.class, "color");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
    }

    @Override
    public void loop() {

        if (((DistanceSensor) color).getDistance(DistanceUnit.CM) <=5) {

            if(color.blue() > 100){
                Slide.setPower(1);
            }
            if(color.red() > 100){
                Slide.setPower(1);
            }
        else {
            Slide.setPower(0);
            }
        }


        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) color).getDistance(DistanceUnit.CM));


        telemetry.addData("red: ", color.red()); //checking for colors
        telemetry.addData("blue: ", color.blue()); //checking for colors
        telemetry.addData("alpha:/light ", color.alpha()); //the amount of light is alpha
        telemetry.update();
    }
}
