package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingsPipeline extends OpenCvPipeline {

    public RingsPipeline() {

    }

    Mat hsv = new Mat();
    Mat orange = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsv, hsv, 0);
        Imgproc.threshold(hsv, orange, 20, 40, Imgproc.THRESH_BINARY_INV);

        return orange;
    }
}
