package org.firstinspires.ftc.teamcode;

import android.os.strictmode.ImplicitDirectBootViolation;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
Created by Ryan Su over the course of several random dates in 2021
 */
public class BackboardPipeline extends OpenCvPipeline {

    Color color;
    Constants constants;
    int[] left = {0, 0};
    int[] right = {0, 0};
    double[] rgbL;
    double[] rgbR;

    Mat hsv = new Mat();
    Mat noWhite = new Mat();
    Mat redMat = new Mat();
    Mat redThreshold = new Mat();

    public BackboardPipeline(Color color) {
        this.color = color;
    }

    @Override
    public Mat processFrame(Mat input) {

//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//        Core.extractChannel(hsv, hsv, 0);
//        Imgproc.threshold(hsv, noWhite, 0, 30, Imgproc.THRESH_TOZERO);
//        Core.extractChannel(input, redMat, 0);
//        Imgproc.threshold(redMat, redThreshold, 150, 255, Imgproc.THRESH_BINARY);

        int width = 240;
        int height = 320;
        int rowCol1 = width + height;
        int rowCol2 = width + height;
        int row1 = 0;
        int row2 = 0;
        int col1 = 0;
        int col2 = 0;

        if (input.get(0, 0) == null)
            return input;
        for (int i = 0; i < height; i+= 3) {
            for (int j = 0; j < width; j+= 3) {
                if (i + j < rowCol1 && checkRed(input.get(i, j))) {
                    row1 = (int)j;
                    col1 = (int)i;
                    rowCol1 = i + j;
                    if (i + 10 < height)
                        height = i + 10;
                }
                else if (i - j < rowCol2 && checkRed(input.get(i, j))) {
                    row2 = (int)j;
                    col2 = (int)i;
                    rowCol2 = i - j;
                    if (i + 10 < height)
                        height = i + 10;
                }
            }
        }

        rgbL = input.get(col1, row1);
        rgbR = input.get(col2, row2);

        Imgproc.circle(input, new Point(row1, col1), 10, new Scalar(0, 255, 0, 0));
        Imgproc.circle(input, new Point(row2, col2), 10, new Scalar(0, 255, 0, 0));

        left[0] = (int)row1;
        left[1] = (int)col1;

        right[0] = (int)row2;
        right[1] = (int)col2;

        Imgproc.circle(input, new Point(getX(), getY()), 10, new Scalar(255, 255, 255, 0));

        return input;
    }
    public boolean checkRed(double[] rgb) {
        return rgb[0] > (rgb[1] + rgb[2]) * 0.8 && rgb[1] + rgb[2] < 200 && rgb[0] > 130;
    }

    public int getX() {
        return (right[0] + left[0]) / 2;
    }
    public int getY() {
        return (right[1] + left[1]) / 2;
    }
    public int getHeight() {
        return right[1] - left[1];
    }
    public int getWidth() {
        return right[0] - left[0];
    }
    public int[] getLeft() {
        return left;
    }
    public int[] getRight() {
        return right;
    }
    public double[] getRGBRight() {
        return rgbR;
    }
    public double[] getRGBLeft() {
        return rgbL;
    }

    public boolean detected() {
        return (right[0] + left[0] + right[1] + left[1] != 0);
    }
}
