package org.firstinspires.ftc.teamcode.Camera.Childrens.PipeLines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.Parents.Pipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

/*
 * Пример конвейера обработки изображений, который будет запускаться при получении каждого кадра с камеры.
 * Обратите внимание, что методprocessFrame() вызывается последовательно из рабочего потока фрейма —
 * то есть новый кадр камеры не появится, пока вы еще обрабатываете предыдущий.
 * Другими словами, методprocessFrame() никогда не будет вызываться несколько раз одновременно.
 *
 * Однако рендеринг обработанного изображения в окне просмотра выполняется параллельно с
 * Фреймовый рабочий поток. То есть количество времени, необходимое для рендеринга изображения в
 * Область просмотра НЕ влияет на количество кадров в секунду, которые может обрабатывать ваш конвейер.
 *
 * ВАЖНОЕ ПРИМЕЧАНИЕ: этот конвейер НЕ вызывается в вашем потоке OpMode. Он вызывается на
 * Фреймовый рабочий поток. В подавляющем большинстве случаев это не должно быть проблемой. Однако,
 * если вы делаете что-то странное и вам нужно синхронизировать это с потоком OpMode,
 * тогда вам нужно будет это учитывать соответствующим образом.
 */
public class PipelineReDRight extends Pipeline {
    public PipelineReDRight(OpenCvCamera webcam, Telemetry telemetry){
        super(webcam, telemetry);
        this.webcam = webcam;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);//из RGB в YCrCb
        telemetry.addLine("Pipeline running");

        Rect leftRect = new Rect(0,280, 100, 100);
        Rect midleRect = new Rect(240,280, 300, 100);

        Core.extractChannel(YCbCr, YCbCr, 2);//оставляем только КРАСНЫЙ цвет по политре YCbCr

        Imgproc.threshold(YCbCr, YCbCr, 120, 255, Imgproc.THRESH_BINARY_INV);

        leftCrop = YCbCr.submat(leftRect);
        midleCrop = YCbCr.submat(midleRect);


        double valueleft = Core.sumElems(leftCrop).val[0] / leftRect.area() / 255;
        double valuemiddle = Core.sumElems(midleCrop).val[0] / midleRect.area() / 255;

        leftCrop.release();
        midleCrop.release();


        //Процент нужного цвета в рамке
        telemetry.addData("RED percentage in left", Math.round(valueleft * 100) + "%");
        telemetry.addData("RED percentage in middle", Math.round(valuemiddle * 100) + "%");

        Imgproc.rectangle(YCbCr, leftRect, rectColor, 2);
        Imgproc.rectangle(YCbCr, midleRect, rectColor, 2);

        if(Math.round(valueleft * 100) > 10 && Math.round(valueleft * 100)> Math.round(valuemiddle * 100)){
            telemetry.addLine("Position: left");
        }else if(Math.round(valuemiddle * 100) > 10 && Math.round(valuemiddle * 100) > Math.round(valueleft * 100) ){
            telemetry.addLine("Position: middle");
        }else {
            telemetry.addLine("Position: right");
        }

        telemetry.update();

        return (YCbCr);
    }
}
