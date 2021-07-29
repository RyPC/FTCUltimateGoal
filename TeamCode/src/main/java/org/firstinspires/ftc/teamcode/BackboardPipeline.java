package org.firstinspires.ftc.teamcode;

import android.os.strictmode.ImplicitDirectBootViolation;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Vector;

/*
Created by Ryan Su over the course of several random dates in 2021
 */
public class BackboardPipeline extends OpenCvPipeline {

    Color color;
    Constants constants;
    int[] left = {0, 0};
    int[] right = {0, 0};
    public double[] rgbL;
    public double[] rgb;
    public double[] rgbR;

    Mat hsv = new Mat();
    Mat redMat = new Mat();

    public BackboardPipeline(Color color) {
        this.color = color;
    }

    @Override
    public Mat processFrame(Mat input) {

        //testing different vision processes
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//        Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(15, 255, 255), redMat);
//        Mat lines = new Mat();
//        Imgproc.Canny(redMat, lines, 50, 200, 3, false);
//        Imgproc.HoughLines(lines, lines, 1, Math.PI/180, 50);
//        for (int x = 0; x < lines.rows(); x++) {
//            double rho = lines.get(x, 0)[0],
//                    theta = lines.get(x, 0)[1];
//            double a = Math.cos(theta), b = Math.sin(theta);
//            double x0 = a * rho, y0 = b * rho;
//            Point pt1 = new Point(Math.round(x0 + 1000 * (-b)), Math.round(y0 + 1000 * (a)));
//            Point pt2 = new Point(Math.round(x0 - 1000 * (-b)), Math.round(y0 - 1000 * (a)));
//            Imgproc.line(input, pt1, pt2, new Scalar(255, 255, 0), 3, Imgproc.LINE_AA, 0);
//        }



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
                if (i + j < rowCol1 && (color == Color.RED ? checkRed(input.get(i, j)) : checkBlue(input.get(i, j)))) {
                    row1 = (int)j;
                    col1 = (int)i;
                    rowCol1 = i + j;
                    if (i + 30 < height)
                        height = i + 30;
                }
                else if (i - j < rowCol2 && (color == Color.RED ? checkRed(input.get(i, j)) : checkBlue(input.get(i, j)))) {
                    row2 = (int)j;
                    col2 = (int)i;
                    rowCol2 = i - j;
                    if (i + 30 < height)
                        height = i + 30;
                }
            }
        }

        rgbL = input.get(col1, row1);
        rgbR = input.get(col2, row2);
        rgb = input.get((col1 + col2) / 2, (row1 + row2) / 2);

        if (detected()) {
            Imgproc.circle(input, new Point(row1, col1), 10, new Scalar(0, 255, 0, 0));
            Imgproc.circle(input, new Point(row2, col2), 10, new Scalar(0, 255, 0, 0));
        }

        left[0] = (int)row1;
        left[1] = (int)col1;

        right[0] = (int)row2;
        right[1] = (int)col2;

        Imgproc.circle(input, new Point(getX(), getY()), 10, new Scalar(255, 255, 255, 0));

        return input;
    }
    public boolean checkRed(double[] rgb) {
        return rgb[0] > (rgb[1] + rgb[2]) * 0.75 && rgb[0] > 170;
    }
    public boolean checkBlue(double[] rgb) {
        return rgb[2] > (rgb[0] + rgb[1]) * 0.7 && rgb[2] > 150;
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
    public double[] getRGB() {
        return rgb;
    }
    public double[] getRGBLeft() {
        return rgbL;
    }
    public double getAngle() {
        return (double) getHeight() / getWidth() * 90 - 4;
    }

    public boolean detected() {
        return (right[0] + left[0] + right[1] + left[1] != 0) && getWidth() > 10 /*&& (color == Color.RED ? checkRed(getRGB()) : checkBlue(getRGB()))*/;
    }
}
