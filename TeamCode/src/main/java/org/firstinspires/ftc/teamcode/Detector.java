package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//Disabled
public class Detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat(); //Объявляем матрицу
    public enum Location {
        BLUE,
        YELLOW,
        STRIPES
    }
    private static Location location;

    public Detector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        //Рамки
        Rect r1 = new Rect(350, 280, 100, 150);
//        final Rect r2 = new Rect(
//                new Point(input.cols()*280f/330f, input.rows()*195f/330f),
//                new Point(input.cols()*330f/330f, input.rows()*270f/330f));

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb); //RGB в YCrCb
        Core.extractChannel(mat, mat, 2); //Синий цвет

        //Чем меньше синего - тем лучше
        //Больше thresh - менее строгое определение
        Imgproc.threshold(mat, mat, 120, 255, Imgproc.THRESH_BINARY_INV);

        Mat mr1 = mat.submat(r1);

        double value = Core.sumElems(mr1).val[0] / r1.area() / 255;

        mr1.release();

//        Mat mr2 = mat.submat(r2);
//
//        double value2 = Core.sumElems(mr2).val[0] / r2.area() / 255;
//
//        mr2.release();

        //Процент нужного цвета в рамке
        telemetry.addData("Yellow percentage", Math.round(value * 100) + "%");

        Scalar white = new Scalar(255, 255, 255);

        //Рисуем рамки
        Imgproc.rectangle(mat, r1, white, 2);
//        Imgproc.rectangle(mat, r2, white, 2);

//        Определяем локацию
//        В локации не менее 10% нужного цвета
        if (Math.round(value * 100) > 45) {
            location = Location.YELLOW;
        }
        else if (Math.round(value * 100) > 10) {
            location = Location.STRIPES;
        } else {
            location = Location.BLUE;
        }

        telemetry.addLine("Color: " + location);
        telemetry.update();

        return mat;
    }

    //Метод получения локации
    public static Location getLocation() {
        return (Location) location;
    }
}
