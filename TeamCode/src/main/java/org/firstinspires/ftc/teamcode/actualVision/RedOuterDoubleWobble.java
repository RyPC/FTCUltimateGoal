package org.firstinspires.ftc.teamcode.actualVision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.teamcode.BackboardPipeline;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.firstinspires.ftc.teamcode.enums.Side;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvViewport;

import java.util.Arrays;

@Autonomous(name = "Red Outer Double Wobble", group = "Vision")
public class RedOuterDoubleWobble extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;
    OpenCvCamera ringCamera;
    int cameraMonitorViewId;
    @Override

    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        cameraMonitorViewId = robotHardware.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robotHardware.hwMap.appContext.getPackageName());

        //set up ring camera
        ringCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.ringWebcam);

        Pipeline ringPipeline = new Pipeline(Color.BLUE, Side.LEFT);
        ringCamera.setPipeline(ringPipeline);

        ringCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ringCamera.startStreaming(constants.cameraWidth, constants.cameraHeight, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });


        telemetry.addLine("Ready for Start");
        telemetry.update();

        Rings position = Rings.FOUR;
        while (!isStarted() && !isStopRequested()) {
            position = ringPipeline.getPosition();
            telemetry.addData("Position", position);
            telemetry.update();
        }

        ringCamera.setPipeline(null);
        ringCamera.stopStreaming();
        ringCamera.closeCameraDevice();
        ringCamera = null;

        backboardCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.backboardWebcam, cameraMonitorViewId);

        BackboardPipeline pipeline = new BackboardPipeline(Color.BLUE);
        backboardCamera.setPipeline(pipeline);

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

        robotHardware.shooter.setVelocity(constants.shooterPower);
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();

            if (stage != prevStage) {
                elapsedTime.reset();
                prevStage = (int)stage;
            }

            switch (stage) {
                case 0:
                    //wobble goal
                    robotHardware.shooter.setVelocity(constants.shooterPower);
                    robotHardware.drivePower(113, 1);
                    robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                    robotHardware.strafePower(4, 1);
                    robotHardware.strafePower(-11, -1);
                    robotHardware.drivePower(-38, -0.9);
                    robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                    robotHardware.sleep(250);
                    stage++;
                    break;
                case 1:
                case 4:
                case 7:
                case 9:
                    //shoot
                    if (robotHardware.blocker.getPosition() != constants.blockerUp)
                        movement.goToPoint(135, 120, pipeline);
                    if (movement.closeTo(135, 120, 15, pipeline)) {
                        movement.shoot();
                        robotHardware.intakeOn();
                    }
                    if (elapsedTime.milliseconds() > (stage == 1 ? 3000 : 3500)) {
                        stage++;
                        movement.block();
                    }
                    robotHardware.shooter.setVelocity(constants.shooterPower);
                    break;
                case 2:
                    //line up for rings
                    movement.block();
                    movement.goToPoint(94, 178, pipeline);
                    if (elapsedTime.milliseconds() > 1750)
                        stage++;
                    break;
                case 3:
                    //collect stack
                    robotHardware.intakeOn();
                    robotHardware.turnTo(90);
                    robotHardware.drivePower(13, 0.15, 90);
                    robotHardware.intake.setPower(-0.75);
                    robotHardware.drivePower(-4, -1, 90);
                    robotHardware.intakeOn();
                    movement.block();
                    stage++;
                    break;
                case 5:
                    //line up for 2nd wobble
                    robotHardware.intakeOff();
                    movement.goToPoint(141, 212, pipeline, 1);
                    if (elapsedTime.milliseconds() > 2500)
                        stage++;
                    break;
                case 6:
                    //pick up 2nd wobble
                    robotHardware.turnTo(180);
                    robotHardware.wobbleArm.setPosition(constants.armDown);
                    robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                    robotHardware.strafePower(14, 0.3333);
                    robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                    robotHardware.intakeOn();
                    robotHardware.sleep(333);
                    robotHardware.strafePower(10,1);
                    robotHardware.drivePower(10, 1, 0);
                    movement.block();
                    robotHardware.intakeOn();
                    stage++;
                    break;
                case 8:
                    //drive to and deposit wobble
                    robotHardware.drivePower(50, 0.75);
                    robotHardware.strafePower(6, 0.75);
                    robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                    robotHardware.wobbleArm.setPosition(constants.armUp);
                    robotHardware.strafePower(-4, -0.75);
                    robotHardware.drivePower(-40, -1);
                    robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                    stage++;
                    break;
                case 100:
                    //pp-pp-pparkkk
                    robotHardware.intakeOff();
                    movement.goToPoint(153, 90, pipeline);
                    break;
                default:
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
