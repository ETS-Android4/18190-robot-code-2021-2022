package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class OpenCvTestPipeline extends OpenCvPipeline
{

    private static int MAX_BINARY_VALUE = 255;

    private static final int THRESHOLD_VALUE = 0;
    private static final int THRESHOLD_TYPE = 3;



    @Override
    public Mat processFrame(Mat input)
    {
        Mat dst = new Mat();
        Imgproc.threshold(input, dst, THRESHOLD_VALUE, MAX_BINARY_VALUE, THRESHOLD_TYPE);
        return dst;
    }
}
