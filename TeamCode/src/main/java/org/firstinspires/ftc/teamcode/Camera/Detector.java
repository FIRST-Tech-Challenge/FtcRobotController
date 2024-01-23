package org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//@Disabled
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
        final Rect rr1 = new Rect(
                new Point(input.cols()*330f/330f, input.rows()*300f/330f),
                new Point(input.cols()*264/330f, input.rows()*160f/330f));
        final Rect rr2 = new Rect(
                new Point(input.cols()*264f/330f, input.rows()*300f/330f),
                new Point(input.cols()*198f/330f, input.rows()*160f/330f));
        final Rect rc = new Rect(
                new Point(input.cols()*198f/330f, input.rows()*300f/330f),
                new Point(input.cols()*132f/330f, input.rows()*160f/330f));
        final Rect rl2 = new Rect(
                new Point(input.cols()*132f/330f, input.rows()*300f/330f),
                new Point(input.cols()*66f/330f, input.rows()*160f/330f));
        final Rect rl1 = new Rect(
                new Point(input.cols()*66f/330f, input.rows()*300f/330f),
                new Point(input.cols()*0f/330f, input.rows()*160f/330f));

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb); //RGB в YCrCb
        Core.extractChannel(mat, mat, 2); //Фильтруем красный цвет

        //Чем меньше синего — тем лучше
        //Больше thresh — менее строгое определение
        Imgproc.threshold(mat, mat, 116, 255, Imgproc.THRESH_BINARY_INV);

        Mat mr1 = mat.submat(rr1);
        Mat mr2 = mat.submat(rr2);
        Mat mc = mat.submat(rc);
        Mat ml2 = mat.submat(rl2);
        Mat ml1 = mat.submat(rl1);

        double value_r1 = Core.sumElems(mr1).val[0] / rr1.area() / 255;
        double value_r2 = Core.sumElems(mr2).val[0] / rr2.area() / 255;
        double value_c = Core.sumElems(mc).val[0] / rc.area() / 255;
        double value_l2 = Core.sumElems(mr2).val[0] / rl2.area() / 255;
        double value_l1 = Core.sumElems(mr1).val[0] / rl1.area() / 255;

        mr1.release();
        mr2.release();
        mc.release();
        ml1.release();
        ml2.release();

        //Процент нужного цвета в рамке
        //telemetry.addData("RIGHT R", Math.round(value_r1 * 100) + "%");
        //telemetry.addData("RIGHT", Math.round(value_r2 * 100) + "%");
        //telemetry.addData("CENTER", Math.round(value_c * 100) + "%");
        //telemetry.addData("LEFT", Math.round(value_l2 * 100) + "%");
        //telemetry.addData("LEFT L", Math.round(value_l1 * 100) + "%");

        Scalar white = new Scalar(255, 255, 255);
        //Рисуем рамки
        Imgproc.rectangle(mat, rr1, white, 3);
        Imgproc.rectangle(mat, rr2, white, 2);
        Imgproc.rectangle(mat, rc, white, 2);
        Imgproc.rectangle(mat, rl2, white, 2);
        Imgproc.rectangle(mat, rl1, white, 2);
        //Определяем локацию

        /*
        if (Math.round(value_c * 100) > Math.round(value_l * 100) && Math.round(value_c * 100) > Math.round(value_r * 100)) {
            location = Location.CENTER;
        }
        else if (Math.round(value_l * 100) > Math.round(value_r * 100)) {
            location = Location.LEFT;
        }
        else {
            location = Location.RIGHT;
        }

        telemetry.addLine("Location: " + location);
        */

        //telemetry.update();

        return mat;
    }


    //Метод получения локации
    public static Location getLocation() {
        return (Location) location;
    }
}