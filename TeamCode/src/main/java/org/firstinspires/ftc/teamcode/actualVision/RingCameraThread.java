package org.firstinspires.ftc.teamcode.actualVision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BackboardPipeline;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.RingsPipeline;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RingCameraThread extends Thread {

    Color color;
    LinearOpMode op;
    RobotHardware robotHardware;
    Telemetry telemetry;
    Constants constants = new Constants();
    Rings position = Rings.FOUR;
    OpenCvCamera camera;
    Pipeline pipeline;

    public RingCameraThread(LinearOpMode op, Telemetry telemetry, Color color) {
        this.op = op;
        this.telemetry = telemetry;
        this.color = color;
    }

    @Override public void run() {

        camera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.ringWebcam);

        Pipeline pipeline = new Pipeline(color);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(constants.cameraWidth, constants.cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }
        });

        while (!Thread.currentThread().isInterrupted() && !op.isStarted() && !op.isStopRequested()) {
            position = pipeline.getPosition();
        }
    }

    public Rings getPosition() {
        return position;
    }
}
