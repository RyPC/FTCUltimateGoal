package org.firstinspires.ftc.teamcode.actualVision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BackboardPipeline;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.nio.channels.Pipe;
import java.util.Arrays;

@Autonomous(name = "Blue 4 Ring", group = "Vision")
public class Blue4Ring extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;
    OpenCvCamera ringCamera;
    int shooterPower = constants.shooterPower + 45;
    @Override

    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        backboardCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.backboardWebcam);
        ringCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.ringWebcam);

        BackboardPipeline pipeline = new BackboardPipeline(Color.RED);
        Pipeline ringPipeline = new Pipeline(Color.RED);

        backboardCamera.setPipeline(pipeline);
        ringCamera.setPipeline(ringPipeline);

        ringCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ringCamera.startStreaming(constants.cameraWidth, constants.cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }
        });
        telemetry.addLine("Ready for Start");
        telemetry.update();

        Rings position;
        while (!isStarted() && !isStopRequested()) {
            if (pipeline.detected()) {
                movement.block();
                telemetry.addLine("Pos: [" + pipeline.getX() + ", " + pipeline.getY() + "]");

                int[] left = pipeline.getLeft();
                telemetry.addLine("Left: [" + left[0] + ", " + left[1] + "]");
                int[] right = pipeline.getRight();
                telemetry.addLine("Right: [" + right[0] + ", " + right[1] + "]");
                telemetry.addLine("Size: [" + pipeline.getWidth() + ", " + pipeline.getHeight() + "]");
                telemetry.addLine("RGB");
                telemetry.addData("Left", Arrays.toString(pipeline.getRGBLeft()));
                telemetry.addData("Right", Arrays.toString(pipeline.getRGBRight()));
            }
            position = ringPipeline.getPosition();
            telemetry.addData("Position", position);
            telemetry.update();

        }

        //stop ring camera and start backboard camera
        ringCamera.stopStreaming();
        backboardCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                backboardCamera.startStreaming(constants.cameraWidth, constants.cameraHeight, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        ElapsedTime elapsedTime = new ElapsedTime();
        ElapsedTime totalTime = new ElapsedTime();
        totalTime.reset();

        movement.block();
        int stage = 0;
        int prevStage = -1;

        robotHardware.shooter.setVelocity(shooterPower);
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();

            if (stage != prevStage) {
                elapsedTime.reset();
                prevStage = (int)stage;
            }

            switch (stage) {
                case 0:
                    //wobble goal
                    robotHardware.shooter.setVelocity(shooterPower);
                    robotHardware.drivePower(109, 1);
                    robotHardware.turnTo(90);
                    robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                    robotHardware.wobbleArm.setPosition(constants.armUp);
                    robotHardware.strafePower(-60, -0.75);
                    robotHardware.turnTo(0);
                    robotHardware.angleAdjustment = 20;
                    stage++;
                    break;
                case 1:
                case 4:
                case 7:
                    //shoot
                    if (robotHardware.blocker.getPosition() != constants.blockerUp)
                        movement.goToPoint(127, 132, pipeline);
                    if (movement.closeTo(127, 132, 10, pipeline)) {
                        movement.shoot(shooterPower);
                        robotHardware.intakeOn();
                    }
                    if (elapsedTime.milliseconds() > 3000) {
                        stage++;
                        movement.block();
                    }
                    robotHardware.shooter.setVelocity(shooterPower);
                    break;
                case 2:
                case 5:
                    //line up stack
                    robotHardware.intake.setPower(-1);
                    movement.goToPoint(106, 172, pipeline);
                    if (elapsedTime.milliseconds() > 2000)
                        stage++;
                    break;
                case 3:
                case 6:
                    //eat stack(yum!)
                    robotHardware.intakeOn();
                    robotHardware.turnTo(-70);
                    robotHardware.drivePower((stage == 3 ? 18 : 24), 0.25, -70);
                    robotHardware.drivePower(-4, -1, -70);
                    robotHardware.intakeOn();
                    movement.block();
                    stage++;
                    break;
                case 100:
                    //pp-pp-pparkkk
                    robotHardware.intakeOff();
                    movement.goToPoint(151, 118, pipeline);
                    break;
                default:
                    robotHardware.shooter.setPower(0);
                    stage++;
                    break;
            }

            if (totalTime.milliseconds() > 29000)
                stage = 10;
            movement.setPowers();
            telemetry.addData("Stage", stage);
            telemetry.addData("Detected", pipeline.detected());
            telemetry.addData("Shooter Power", robotHardware.shooter.getPower());
//            int[] left = pipeline.getLeft();
//            telemetry.addLine("Left: [" + left[0] + ", " + left[1] + "]");
//            int[] right = pipeline.getLeft();
//            telemetry.addLine("Right: [" + right[0] + ", " + right[1] + "]");
//            telemetry.addLine("Pos: [" + pipeline.getX() + ", " + pipeline.getY() + "]");
            telemetry.addData("time", totalTime.milliseconds());
//            telemetry.addLine("RGB");
//            telemetry.addData("Right", Arrays.toString(pipeline.getRight()));
//            telemetry.addData("Left", Arrays.toString(pipeline.getLeft()));
            telemetry.update();
        }
    }

}
