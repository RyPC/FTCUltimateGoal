package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Pee hole", group = "Vision")
public class PeePee extends LinearOpMode {

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
        telemetry.addLine("Scanning for board");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (pipeline.detected()) {
                telemetry.addLine("Pos: [" + pipeline.getX() + ", " + pipeline.getY() + "]");

                int[] left = pipeline.getLeft();
                telemetry.addLine("Left: [" + left[0] + ", " + left[1] + "]");
                int[] right = pipeline.getRight();
                telemetry.addLine("Right: [" + right[0] + ", " + right[1] + "]");
                telemetry.addLine("Size: [" + pipeline.getWidth() + ", " + pipeline.getHeight() + "]");
                telemetry.update();

            }

        }

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();

//            double[] rgb = pipeline.getRGB();
//            if (rgb != null) {
//                telemetry.addLine("RGB: [" + rgb[0] + ", " + rgb[1] + ", " + rgb[2] + "]");
//                telemetry.update();
//            }

            robotHardware.shooter.setVelocity(constants.shooterPower);
            if (elapsedTime.milliseconds() < 1250)
                movement.goToPoint(189, 183, pipeline);
            else if (elapsedTime.milliseconds() < 7000) {
                movement.goToPoint(140, 140, pipeline);
                if (movement.closeTo(140, 140, 6, pipeline) && elapsedTime.milliseconds() > 5000)
                    movement.shoot();
            }
            else {
                movement.goToPoint(189, 183, pipeline);
                movement.shooterOff();
            }

            movement.setPowers();
            telemetry.addData("Detected", pipeline.detected());
            int[] left = pipeline.getLeft();
            telemetry.addLine("Left: [" + left[0] + ", " + left[1] + "]");
            int[] right = pipeline.getLeft();
            telemetry.addLine("Right: [" + right[0] + ", " + right[1] + "]");
            telemetry.addLine("Pos: [" + pipeline.getX() + ", " + pipeline.getY() + "]");
            telemetry.addData("time", elapsedTime.milliseconds());
            telemetry.update();
        }
    }

}
