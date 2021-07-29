//Pipeline for ring stack during init
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.firstinspires.ftc.teamcode.enums.Side;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

public class Pipeline  extends OpenCvPipeline {

    Constants constants = new Constants();

    Color color;
    Side side;
    double[][] RGB = {{0, 0, 0}, {0, 0, 0}};
    boolean one = false;
    boolean four = false;

    public Pipeline(Color color, Side side) {
        //change values based on the side of the field?
        this.color = color;
        this.side = side;
        if (side == Side.LEFT) {
            constants.ringWidth = 310;
        }
        else if (side == Side.RIGHT) {
            constants.ringWidth = 10;
        }
    }


    @Override
    public Mat processFrame(Mat input) {
        //scan ring 1
        RGB[0] = input.get(constants.heightOne, constants.ringWidth);
        Imgproc.circle(input, new Point(constants.ringWidth, constants.heightOne), 10, new Scalar(0, 255, 0, 0));

        //scan ring 2
        RGB[1] = input.get(constants.heightFour, constants.ringWidth);
        Imgproc.circle(input, new Point(constants.ringWidth, constants.heightFour), 10, new Scalar(0, 0, 255, 0));

        //process ring values
        one = checkOrange(RGB[0]);
        four = checkOrange(RGB[1]);
        return input;
    }

    //checks if the given pixel is "orange"
    public boolean checkOrange(double[] rgb) {
        return rgb[0] > 90 && rgb[1] > rgb[2] * 1.4;
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
