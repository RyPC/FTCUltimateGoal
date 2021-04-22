package org.firstinspires.ftc.teamcode.visionAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BackboardPipeline;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@Autonomous(name = "Reset", group = "Vision")
public class Reset extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        BackboardPipeline pipeline = new BackboardPipeline(Color.RED);
        robotHardware.camera.setPipeline(pipeline);

        robotHardware.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotHardware.camera.startStreaming(constants.width, constants.height, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
        telemetry.addLine("Ready for Start");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (pipeline.detected()) {
                telemetry.addLine("Pos: [" + pipeline.getX() + ", " + pipeline.getY() + "]");

                int[] left = pipeline.getLeft();
                telemetry.addLine("Left: [" + left[0] + ", " + left[1] + "]");
                int[] right = pipeline.getRight();
                telemetry.addLine("Right: [" + right[0] + ", " + right[1] + "]");
                telemetry.addLine("Size: [" + pipeline.getWidth() + ", " + pipeline.getHeight() + "]");
                telemetry.addLine("RGB");
                telemetry.addData("Left", Arrays.toString(pipeline.getRGBLeft()));
                telemetry.addData("Right", Arrays.toString(pipeline.getRGBRight()));
                telemetry.addData("angle", pipeline.getAngle());
                telemetry.update();

            }

        }

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        robotHardware.blocker.setPosition(constants.blockerDown);
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();
            movement.goToPoint(203,224, pipeline);
            movement.setPowers();
            telemetry.addData("Detected", pipeline.detected());
            int[] left = pipeline.getLeft();
            telemetry.addLine("Left: [" + left[0] + ", " + left[1] + "]");
            int[] right = pipeline.getLeft();
            telemetry.addLine("Right: [" + right[0] + ", " + right[1] + "]");
            telemetry.addLine("Pos: [" + pipeline.getX() + ", " + pipeline.getY() + "]");
            telemetry.addData("time", elapsedTime.milliseconds());
            telemetry.addLine("RGB");
            telemetry.addData("Right", Arrays.toString(pipeline.getRight()));
            telemetry.addData("Left", Arrays.toString(pipeline.getLeft()));
            telemetry.addData("angle", pipeline.getAngle());
            telemetry.update();
        }
    }

}
