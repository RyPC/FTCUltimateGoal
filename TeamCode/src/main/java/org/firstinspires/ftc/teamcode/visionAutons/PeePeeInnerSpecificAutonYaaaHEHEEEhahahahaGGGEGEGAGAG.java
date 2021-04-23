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

@Autonomous(name = "PeePeeInnerSpecificAutonYaaaHEHEEEhahahahaGGGEGEGAGAG", group = "Vision")
public class PeePeeInnerSpecificAutonYaaaHEHEEEhahahahaGGGEGEGAGAG extends LinearOpMode {

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
        ElapsedTime totalTime = new ElapsedTime();
        totalTime.reset();

        movement.block();
        robotHardware.shooter.setVelocity(constants.shooterPower);
        int stage = 0;
        int prevStage = -1;

        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();
            if (stage != prevStage) {
                elapsedTime.reset();
                prevStage = (int)stage;
            }

            switch (stage) {
                case 0:
                    //wobble goal
                    robotHardware.drivePower(116, 0.75, false, 0, 0);
                    robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                    robotHardware.strafeTo(-4);
                    robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                    robotHardware.drivePower(-45, -0.9);
                    stage++;
                    break;
                case 1:
                case 4:
                    //shoot
                    if (robotHardware.blocker.getPosition() != constants.blockerUp)
                        movement.goToPoint(135, 135, pipeline);
                    if (movement.closeTo(135, 135, 10, pipeline))
                        movement.shoot();
                    if (elapsedTime.milliseconds() > 3500)
                        stage++;
                    break;
                case 2:
                    //line up for rings
                    movement.block();
                    movement.goToPoint(94, 178, pipeline);
                    if (elapsedTime.milliseconds() > 2000)
                        stage++;
                    break;
                case 3:
                    //collect stack
                    robotHardware.intakeOn();
                    robotHardware.turnTo(90);
                    robotHardware.drivePower(11, 0.15, 90);
                    robotHardware.intake.setPower(-1);
                    robotHardware.drivePower(-3, -1, 90);
                    robotHardware.intakeOn();
                    movement.block();
                    stage++;
                    break;
                case 5:
                    //line up for stack rings
                    robotHardware.intakeOff();
                    movement.goToPoint(152, 213, pipeline);
                    if (elapsedTime.milliseconds() > 1750)
                        stage++;
                    break;
                case 6:
                    //pick up other wobble
                    robotHardware.turnTo(180);
                    robotHardware.wobbleArm.setPosition(constants.armDown);
                    robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                    robotHardware.strafePower(14, 0.25);
                    robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                    robotHardware.sleep(333);
                    stage++;
                    break;
                case 7:
                    //idek
                    robotHardware.blocker.setPosition(constants.blockerDown);
                    robotHardware.intakeOn();
                    movement.goToPoint(135,135,pipeline);
                    if(elapsedTime.milliseconds() > 2500)
                        stage = 10;
                case 10:
                    //pp
                    movement.goToPoint(153, 90, pipeline);
                    break;

            }

//            if (elapsedTime.milliseconds() > 8000)
//                stage++;
            if (totalTime.milliseconds() == 28)
                stage = 10;
            movement.setPowers();
            telemetry.addData("Stage", stage);
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
