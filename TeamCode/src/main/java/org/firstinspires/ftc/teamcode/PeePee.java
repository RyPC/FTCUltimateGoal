package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

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
                telemetry.update();

            }

        }

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        robotHardware.blocker.setPosition(constants.blockerDown);
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();

            robotHardware.shooter.setVelocity(constants.shooterPower);
            if (elapsedTime.milliseconds() < 500)
                robotHardware.drivePower(50, 0.75, false, 0, 0);
            else if (elapsedTime.milliseconds() < 7000) {
                if (robotHardware.blocker.getPosition() != constants.blockerUp)
                    movement.goToPoint(135, 135, pipeline);
                if (movement.closeTo(135, 135, 10, pipeline) && elapsedTime.milliseconds() > 5000)
                    movement.shoot();
            }
            else if (elapsedTime.milliseconds() < 10000) {
                robotHardware.blocker.setPosition(constants.blockerDown);
                robotHardware.intakeOff();
                robotHardware.driveTo(58);
                robotHardware.placeWobble();
                robotHardware.strafeTo(-4);
                robotHardware.drivePower(-58, -0.85);
            }
            else if (elapsedTime.milliseconds() < 14500) {
                movement.goToPoint(94, 178, pipeline);
            }
            else if (elapsedTime.milliseconds() < 15000) {
                robotHardware.intakeOn();
                robotHardware.turnTo(90, 0.5);
                robotHardware.drivePower(10, 0.1, 90);
                robotHardware.drivePower(-10, -0.5, 90);
                robotHardware.intakeOff();
                robotHardware.blocker.setPosition(constants.blockerDown);
            }
            else {
                if (robotHardware.blocker.getPosition() != constants.blockerUp) {
                    movement.goToPoint(135, 135, pipeline);
                }
                if (movement.closeTo(135, 135, 10, pipeline) && elapsedTime.milliseconds() > 5000)
                    movement.shoot();
                else
                    movement.block();
            }

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
            telemetry.update();
        }
    }

}
