package org.firstinspires.ftc.teamcode.visionAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BackboardPipeline;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.RobotHardware;
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
        robotHardware.shooter.setVelocity(constants.shooterPower);
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();
            if (elapsedTime.milliseconds() < 500)
                //move forward
                robotHardware.drivePower(50, 0.75, false, 0, 0);
            else if (elapsedTime.milliseconds() < 2500)
                //goto shooting line to deposit wobble
                movement.goToPoint(135, 135, pipeline);
            else if (elapsedTime.milliseconds() < 3500) {
                //deposit wobble goal
                robotHardware.blocker.setPosition(constants.blockerDown);
                robotHardware.intakeOff();
                robotHardware.drivePower(54, 0.75);
                robotHardware.placeWobble();
                robotHardware.strafeTo(-4);
                robotHardware.drivePower(-40, -1);
            }
            else if (elapsedTime.milliseconds() < 9250) {
                //shoot
                if (robotHardware.blocker.getPosition() != constants.blockerUp)
                    movement.goToPoint(135, 135, pipeline);
                if (movement.closeTo(135, 135, 10, pipeline))
                    movement.shoot();
            }

            else if (elapsedTime.milliseconds() < 11000) {
                //line up to collect stack
                movement.block();
                movement.goToPoint(94, 178, pipeline);
            }
            else if (elapsedTime.milliseconds() < 11500) {
                //collect stack
                robotHardware.intakeOn();
                robotHardware.turnTo(90, 0.5);
                robotHardware.drivePower(11, 0.15, 90);
                robotHardware.drivePower(-3, -1, 90);
                robotHardware.intakeOff();
                movement.block();
            }
            else if (elapsedTime.milliseconds() < 17750){
                //shoot again
                if (robotHardware.blocker.getPosition() != constants.blockerUp) {
                    movement.goToPoint(135, 135, pipeline);
                }
                if (movement.closeTo(135, 135, 10, pipeline))
                    movement.shoot();

            }
            else if (elapsedTime.milliseconds() < 19000) {
                //get 2nd wobble
                movement.block();
                robotHardware.strafeTo(-34);
                robotHardware.turnTo(-130);
                robotHardware.intakeOn();
                robotHardware.driveTo(76, -130);
                robotHardware.intakeOff();
                robotHardware.wobbleArm.setPosition(constants.armDown);
                robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                robotHardware.sleep(500);
                robotHardware.strafeTo(9, -130);
                robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                robotHardware.sleep(500);
                robotHardware.turnTo(0);
            }
            else {
                //shoot yet again
                if (robotHardware.blocker.getPosition() != constants.blockerUp) {
                    movement.goToPoint(135, 135, pipeline);
                }
                if (movement.closeTo(135, 135, 10, pipeline))
                    movement.shoot();
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
