package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

public class Pipeline  extends OpenCvPipeline {

    Constants constants = new Constants();

    double[][] RGB = {{0, 0, 0}, {0, 0, 0}};
    boolean one = false;
    boolean four = false;

    public Pipeline(Color color) {
        //change values based on the side of the field
        if (color == Color.RED) {
            //stuff idk
        }
        else {

        }
    }


    @Override
    public Mat processFrame(Mat input) {
        //scan ring 1
        RGB[0] = input.get(constants.heightOne / 2, constants.widthOne / 2);
        Imgproc.circle(input, new Point(constants.widthOne / 2.0, constants.heightOne / 2.0), 10, new Scalar(0, 255, 0, 0));

        //scan ring 2
        RGB[1] = input.get(constants.heightFour / 2, constants.widthFour / 2);
        Imgproc.circle(input, new Point(constants.widthFour / 2.0, constants.heightFour / 2.0), 10, new Scalar(0, 0, 255, 0));

        //process ring values
        double sensitivity = 2.5;
        one = (getRed(Rings.ONE) + getGreen(Rings.ONE)) >= getBlue(Rings.ONE) * sensitivity;
        four = (getRed(Rings.FOUR) + getGreen(Rings.FOUR)) >= getBlue(Rings.FOUR) * sensitivity;

        return input;
    }

    public int getRed(Rings rings) {
        return (int) (rings == Rings.ONE ? RGB[0][0] : RGB[1][0]);
    }
    public int getGreen(Rings rings) {
        return (int) (rings == Rings.ONE ? RGB[0][1] : RGB[1][1]);
    }
    public int getBlue(Rings rings) {
        return (int) (rings == Rings.ONE ? RGB[0][2] : RGB[1][2]);
    }

    public Rings getPosition() {
        return four ? Rings.FOUR : one ? Rings.ONE : Rings.ZERO;
    }

}
