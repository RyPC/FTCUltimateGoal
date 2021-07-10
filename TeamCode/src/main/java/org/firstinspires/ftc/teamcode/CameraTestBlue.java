package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@TeleOp(name = "camera test blue", group = "TeleOp")
//@Disabled
public class CameraTestBlue extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    TeleOpControls teleOpControls = new TeleOpControls(this, robotHardware, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        BackboardPipeline pipeline = new BackboardPipeline(Color.BLUE);
        robotHardware.camera.setPipeline(pipeline);

        robotHardware.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotHardware.camera.startStreaming(constants.width, constants.height, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        telemetry.addLine("Camera Ready");
        telemetry.update();

        while (!opModeIsActive() && !isStarted() && !isStopRequested()) {
            if (pipeline.detected()) {
                telemetry.addLine("Pos: [" + pipeline.getX() + ", " + pipeline.getY() + "]");

                int[] left = pipeline.getLeft();
                telemetry.addLine("Left: [" + left[0] + ", " + left[1] + "]");
                int[] right = pipeline.getRight();
                telemetry.addLine("Right: [" + right[0] + ", " + right[1] + "]");
                telemetry.addLine("Size: [" + pipeline.getWidth() + ", " + pipeline.getHeight() + "]");
                telemetry.addLine("RGB: " + Arrays.toString(pipeline.getRGBLeft()));

            }
            else
                telemetry.addLine("No backboard detected");

            telemetry.update();
        }


    }
}
