package org.firstinspires.ftc.teamcode.skills;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by sjeltuhin on 9/26/17.
 */

public class ColorCracker {
    NormalizedColorSensor colorSensor;
    ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap hardwareMap){
        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        toggleLight(false);
    }

    private void toggleLight(boolean on){
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }


    public DetectedColor detectColor(Telemetry telemetry, float timeout) {
        toggleLight(true);
        // values is a reference to the hsvValues array.
        DetectedColor dc = DetectedColor.NONE;
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        runtime.reset();
        NormalizedRGBA colors = null;
        boolean stop = false;
        while(!stop) {
                    // Read the sensor
            colors = colorSensor.getNormalizedColors();

            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);

            /** We also display a conversion of the colors to an equivalent Android color integer.
             * @see Color */
            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));

            // Balance the colors. The values returned by getColors() are normalized relative to the
            // maximum possible values that the sensor can measure. For example, a sensor might in a
            // particular configuration be able to internally measure color intensity in a range of
            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
            // intensities of the colors are likely what is most interesting. Here, for example, we boost
            // the signal on the colors while maintaining their relative balance so as to give more
            // vibrant visual feedback on the robot controller visual display.
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            telemetry.update();
            if(colors.red > 0.2 && colors.green > 0.2 && colors.blue < 0.2){
                dc = DetectedColor.Yellow;
            }
            else if (colors.red > 0.2 && colors.green > 0.2 && colors.blue > 0.2){
                dc = DetectedColor.White;
            }
            stop = timeout == 0 || (timeout > 0 && runtime.seconds() >= timeout);
        }

        toggleLight(false);
        return dc;
    }
}
