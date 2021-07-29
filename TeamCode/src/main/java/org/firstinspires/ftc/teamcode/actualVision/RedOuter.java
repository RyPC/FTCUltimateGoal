package org.firstinspires.ftc.teamcode.actualVision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import java.nio.channels.Pipe;
import java.util.Arrays;

@Disabled
@Autonomous(name = "Red Outer", group = "Vision")
public class RedOuter extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;
    OpenCvCamera ringCamera;
    int cameraMonitorViewId;
    int shooterPower = constants.shooterPower - 110;
    @Override

    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, true);


        telemetry.addLine("Setting up camera...");
        telemetry.update();

        cameraMonitorViewId = robotHardware.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robotHardware.hwMap.appContext.getPackageName());

        //set up ring camera
        ringCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.ringWebcam);

        Pipeline ringPipeline = new Pipeline(Color.RED, Side.RIGHT);
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

        BackboardPipeline pipeline = new BackboardPipeline(Color.RED);
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

        robotHardware.shooter.setVelocity(shooterPower);
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();

            //reset timer when stage changes
            if (stage != prevStage) {
                elapsedTime.reset();
                prevStage = (int)stage;
            }

            switch (stage) {
                case 0:
                    //wobble goal
                    robotHardware.shooter.setVelocity(shooterPower);
                    switch(position) {
                        case ZERO:
                            robotHardware.drivePower(68, 1);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-11, -1);
                            robotHardware.drivePower(-10, -1);
                            robotHardware.sleep(5000);
                            stage = 8;
                            break;
                        case ONE:
                            robotHardware.drivePower(86, 1);
                            robotHardware.turnTo(180);
                            robotHardware.strafePower(4, 1);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-11, -1);
                            robotHardware.turnTo(0);
                            robotHardware.drivePower(-40, -0.75);
                            robotHardware.sleep(500);
                            stage++;
                            break;
                        case FOUR:
                            robotHardware.shooter.setVelocity(shooterPower);
                            robotHardware.drivePower(113, 1);
                            robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                            robotHardware.strafePower(4, 1);
                            robotHardware.strafePower(-11, -1);
                            robotHardware.drivePower(-38, -0.9);
                            robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                            robotHardware.sleep(250);
                            stage++;
                            break;
                    }
                    break;
                case 1:
                case 5:
                case 8:
                    //shoot
                    if (robotHardware.blocker.getPosition() != constants.blockerUp)
                        movement.goToPoint(124, 124, pipeline, 0.8);
                    if (movement.closeTo(124, 124, 15, pipeline) && robotHardware.getPowers() < 1.25 && movement.angleCloseTo(2.5)) {
                        movement.shoot(shooterPower, (position != Rings.FOUR) ? 0.25 : 0.5);
                    }
                    if (elapsedTime.milliseconds() > ((position == Rings.FOUR) ? 3000 : 6000)) {
                        if (stage == 5 && position == Rings.ONE)
                            stage = 100;
                        else
                            stage++;
                        movement.block();
                    }
                    robotHardware.shooter.setVelocity(shooterPower);
                    break;
                case 2:
                    //back up from wobble goal
                    robotHardware.strafePower(10, 1);
                    stage++;
                    break;
                case 3:
                    //line up for rings #1
                    movement.block();
                    robotHardware.intake(-1);
                    movement.goToPoint(97, 176, pipeline);
                    if (elapsedTime.milliseconds() > 2000)
                        stage++;
                    break;
                case 6:
                    //line up for rings #2
                    movement.block();
                    robotHardware.intake(-1);
                    movement.goToPoint(120, 176, pipeline);
                    if (elapsedTime.milliseconds() > 2000)
                        stage++;
                    break;
                case 4:
                case 7:
                    //collect stack
                    robotHardware.intakeOn();
                    robotHardware.turnTo(90);
                    robotHardware.drivePower((position == Rings.ONE) ? 20 : 11, (stage == 4) ? 0.15 : 0.3, 90);
                    robotHardware.sleep(250);
                    if (stage == 4 && position == Rings.FOUR)
                        robotHardware.intake.setPower(-0.5);
                    robotHardware.drivePower(-4, -0.5, 90);
                    robotHardware.intakeOn();
                    movement.block();
                    robotHardware.turnTo(15);
                    stage++;
                    break;
                case 100:
                    //pp-pp-pparkkk
                    robotHardware.shooter.setVelocity(0);
                    robotHardware.intakeOff();
                    if (totalTime.milliseconds() < 27500) {

                        int x = (int) (Math.round((40.0 * Math.cos(totalTime.milliseconds()  * 2 * Math.PI / 3000)) / 10.0) * 10);
                        int y = (int) (Math.round((20.0 * Math.sin(totalTime.milliseconds()  * 2 * Math.PI / 3000)) / 10.0) * 10);
                        double pos = ((totalTime.milliseconds() / 333) % 2 == 0) ? constants.armDown : constants.armUp;

                        robotHardware.wobbleArm.setPosition(pos);

                        movement.goToPoint(131 + x, 174 + y, pipeline);
                    }
                    else {
                        movement.goToPoint(153, 90, pipeline);
                    }
                    break;
                default:
                    stage++;
                    break;
            }

            if (totalTime.milliseconds() > 27500)
                stage = 100;
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
