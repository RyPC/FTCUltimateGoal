package org.firstinspires.ftc.teamcode.otherAutons;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BackboardPipeline;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOpControls;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "camera test red", group = "TeleOp")
//@Disabled
public class CameraTestRed extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    TeleOpControls teleOpControls = new TeleOpControls(this, robotHardware, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        BackboardPipeline pipeline = new BackboardPipeline(Color.RED);
        robotHardware.backboardCamera.setPipeline(pipeline);

        robotHardware.backboardCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotHardware.backboardCamera.startStreaming(constants.cameraWidth, constants.cameraHeight, OpenCvCameraRotation.SIDEWAYS_LEFT);
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

            }
            else
                telemetry.addLine("No backboard detected");

            telemetry.update();
        }


    }
}