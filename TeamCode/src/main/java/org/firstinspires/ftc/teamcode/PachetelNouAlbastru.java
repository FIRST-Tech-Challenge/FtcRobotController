package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var_Blue.Day_Hhigh;
import static org.firstinspires.ftc.teamcode.Var_Blue.Day_Hlow;
import static org.firstinspires.ftc.teamcode.Var_Blue.Night_Hhigh;
import static org.firstinspires.ftc.teamcode.Var_Blue.Night_Hlow;
import static org.firstinspires.ftc.teamcode.Var_Blue.Day_Shigh;
import static org.firstinspires.ftc.teamcode.Var_Blue.Day_Slow;
import static org.firstinspires.ftc.teamcode.Var_Blue.Night_Shigh;
import static org.firstinspires.ftc.teamcode.Var_Blue.Night_Slow;
import static org.firstinspires.ftc.teamcode.Var_Blue.Day_Vhigh;
import static org.firstinspires.ftc.teamcode.Var_Blue.Day_Vlow;
import static org.firstinspires.ftc.teamcode.Var_Blue.Night_Vhigh;
import static org.firstinspires.ftc.teamcode.Var_Blue.Night_Vlow;
import static org.firstinspires.ftc.teamcode.Var_Blue.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var_Blue.CV_kernel_pult_size;
import static org.firstinspires.ftc.teamcode.Var_Blue.CV_rect_x1;
import static org.firstinspires.ftc.teamcode.Var_Blue.CV_rect_x2;
import static org.firstinspires.ftc.teamcode.Var_Blue.CV_rect_y1;
import static org.firstinspires.ftc.teamcode.Var_Blue.CV_rect_y2;
import static org.opencv.core.CvType.CV_8UC1;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PachetelNouAlbastru extends OpenCvPipeline {
    //stabileste forma detectorului(dreptunghi)
    private final int elementType = Imgproc.CV_SHAPE_RECT;
    //asta e un dreptunghi(Rect = dreptunghi pentru webcam)
    private Rect dreptunghi;
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    //mat = foaie de desen pentru webcam
    public Mat processFrame(Mat in) {
        //face un patrat de latura kernel_pult_size si cu ancora in centru
        Mat element = Imgproc.getStructuringElement(elementType, new Size(2 * CV_kernel_pult_size + 1, 2 * CV_kernel_pult_size + 1),
                new Point(CV_kernel_pult_size, CV_kernel_pult_size));
        //creeaza o copie a imaginii de pe webcam
        Mat original = in.clone();
        //aceasta linie de cod face un dreptunghi cat webcam-ul de mare si negru
        Mat rect = new Mat();
        Mat input = new Mat(in.rows(),in.cols(), CV_8UC1,Scalar.all(0));
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        try {
            //Scalari de HSV(totusi H e jumate din valorile de pe color picker)
            Scalar scalarLowerHSV, scalarUpperHSV;

            //daca e day da valorile de day, daca e night da valorile de night
            if (CV_detectionType == Var_Blue.DetectionTypes.DAY) {
                scalarLowerHSV = new Scalar(Day_Hlow, Day_Slow, Day_Vlow);
                scalarUpperHSV = new Scalar(Day_Hhigh, Day_Shigh, Day_Vhigh);
            }
            else{
                scalarLowerHSV = new Scalar(Night_Hlow, Night_Slow, Night_Vlow);
                scalarUpperHSV = new Scalar(Night_Hhigh, Night_Shigh, Night_Vhigh);
            }

            //asta converteste culorile din input de la RGB la HSV
            Imgproc.cvtColor(in, in, Imgproc.COLOR_RGB2HSV);

            //asta face ca culorile din input sa fie intre scalarLowerHSV si scalarUpperHSV
            Core.inRange(in, scalarLowerHSV, scalarUpperHSV, input);

            //aceasta parte reduce partile neregulate din imagine
            //erode micsoreaza pixelii, dilate mareste pixelii
            Imgproc.erode(input, input, element);
            Imgproc.dilate(input, input, element);
            Imgproc.dilate(input, input, element);
            Imgproc.erode(input, input, element);

            rect = new Mat(input.rows(), input.cols(), input.type(), Scalar.all(0));
            //face un dreptunghi care stabileste zona de detectare
            Imgproc.rectangle(
                    rect,
                    new Point(CV_rect_x1, CV_rect_y1),
                    new Point(CV_rect_x2, CV_rect_y2),
                    new Scalar(255),
                    Imgproc.FILLED
            );
//            Log.d("dimensiuni input:", input.toString());
//            Log.d("dimensiuni rect:", rect.toString());
            //aici se conveerteste culoarea in alb-negru, albul fiind chestiile detectate
            Core.bitwise_and(input, rect, input);

            //asta declara o lista de contururi;
            List<MatOfPoint> contours = new ArrayList<>();
            //input e imaginea, contours este lista de contururi, retr_list doar da toate contururile, chain_approx_simple face ca formele sa fie facute numai din varfurile lor
            Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            //Aici se sorteaza contururile in mod descrescator;
            Collections.sort(contours, new Comparator<MatOfPoint>() {
                @Override
                public int compare(MatOfPoint matOfPoint, MatOfPoint t1) {
                    return (int) (Imgproc.contourArea(t1) - Imgproc.contourArea(matOfPoint));
                }
            });
            //aici se face un dreptunghi din conturul cel mai mare
            if (!contours.isEmpty()) {
                setRect(Imgproc.boundingRect(contours.get(0)));
            }
            else{
                setRect(new Rect());
            }
            //aici se converteste imaginea de la alb-negru la  color inapoi
            Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGBA);

            //reface imaginea originala cu tot cu partea de culoarea potrivita diferita
            Core.bitwise_or(input, original, input);
            //deseneaza toate contururile, con
            // tourldx=-1 inseamna ca sunt desenate TOATE contururile
            Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 4);
            //aici se deseneaza dreeptunghiul care stabileste aria de detectare
            Imgproc.rectangle(
                    input,
                    new Point(CV_rect_x1, CV_rect_y1),
                    new Point(CV_rect_x2, CV_rect_y2),
                    new Scalar(255, 127, 0), 4);

            //asta umple forma pe care a detectat-o
            Imgproc.rectangle(
                    input,
                    getRect(),
                    new Scalar(0, 255, 255), 4);
            //aici se elimina toate contururile din aceste mat-uri;
            original.release();
            rect.release();
        }
        catch (Exception E){
            original.release();
            rect.release();
        }

        //se returneaza input-ul modificat
        return input;
    }
    public void setRect(Rect rect) {
        this.dreptunghi = rect;
    }

    public Rect getRect() {
        return dreptunghi;
    }
}