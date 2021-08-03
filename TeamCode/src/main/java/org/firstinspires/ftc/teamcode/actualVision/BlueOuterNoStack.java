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
import org.firstinspires.ftc.teamcode.enums.Side;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Outer No Stack", group = "Vision")
public class BlueOuterNoStack extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;
    OpenCvCamera ringCamera;
    int cameraMonitorViewId;
    int shooterPower = constants.shooterPower + 45;
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
                    robotHardware.shooter.setVelocity(0);
                    robotHardware.sleep(12500);
                    robotHardware.shooter.setVelocity(shooterPower);
                    robotHardware.sleep(2500);
                    switch (position) {
                        case ZERO:
                            robotHardware.turnTo(0);
                            robotHardware.drivePower(55, 0.5);
                            robotHardware.turnTo(135);
                            robotHardware.placeWobble();
                            robotHardware.sleep(500);
                            robotHardware.strafePower(-12, -0.75);
                            robotHardware.turnTo(0);
                            robotHardware.shooter.setVelocity(shooterPower);
                            break;
                        case ONE:
                            robotHardware.strafePower(-5, -0.5);
                            robotHardware.drivePower(95, 0.5);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-5, -1);
                            robotHardware.drivePower(-30, -1);
                            break;
                        case FOUR:
                            robotHardware.drivePower(109, 0.5);
                            robotHardware.turnTo(135, 0.75);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-6, -1);
                            robotHardware.turnTo(90);
                            robotHardware.strafePower(-57, -1);
                            break;
                    }
                    //make a permanent change to the angle to be at 20 degrees
                    robotHardware.angleAdjustment = 20;
                    robotHardware.turnTo(0);
                    robotHardware.sleep(500);
                    stage++;
                    break;
                case 1:
                    //go to shooter line
                    movement.goToPoint(127, 132, pipeline, 0.8);
                    if (elapsedTime.milliseconds() > 4500) {
                        stage++;
                    }
                    break;
                case 2:
                    //shoot
                    if (robotHardware.blocker.getPosition() != constants.blockerUp)
                        movement.goToPoint(127, 132, pipeline, 0.8);
                    if (movement.closeTo(127, 132, 15, pipeline)) {
                        movement.shoot(shooterPower, 0.25);
                    }
                    if (elapsedTime.milliseconds() > (position == Rings.FOUR && stage != 7 ? 3000 : 5000)) {
                        if (stage == 1) {
                            switch (position) {
                                case ZERO:
                                    stage = 100;
                                    break;
                                case ONE:
                                    stage = 4;
                                case FOUR:
                                    stage++;
                                    break;
                            }
                        }
                        else {
                            stage++;
                        }

                        movement.block();
                    }
                    robotHardware.shooter.setVelocity(shooterPower);

                    break;
                case 100:
                    //pp-pp-pparkkk
                    robotHardware.intakeOff();
                    shooterPower = 0;
                    robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                    robotHardware.wobbleArm.setPosition(constants.armUp);
                    robotHardware.shooter.setPower(0);
                    if (totalTime.milliseconds() < 25000) {

                        int x = (int)(30.0 * Math.cos(totalTime.milliseconds()  * 2 * Math.PI / 3000));
                        int y = (int)(20.0 * Math.sin(totalTime.milliseconds()  * 2 * Math.PI / 3000));

                        robotHardware.wobbleArm.setPosition((totalTime.seconds() % 2.0  == 0) ? constants.armDown : constants.armUp);

                        movement.goToPoint(75 + x, 186 + y, pipeline);

                    }
                    else {
                        movement.goToPoint(position == Rings.ZERO ? 68 : 151, position == Rings.ZERO ? 100 : 118, pipeline, 0.75);
                    }
                    break;
                default:
                    robotHardware.shooter.setPower(0);
                    stage++;
                    break;
            }

            if (totalTime.milliseconds() > 28500)
                stage = 100;
            movement.setPowers(shooterPower);
            telemetry.addData("Stage", stage);
            telemetry.addData("Detected", pipeline.detected());
            telemetry.addData("Shooter Power", robotHardware.shooter.getPower());
//            int[] left = pipeline.getLeft();
//            telemetry.addLine("Left: [" + left[0] + ", " + left[1] + "]");
//            int[] right = pipeline.getLeft();
//            telemetry.addLine("Right: [" + right[0] + ", " + right[1] + "]");
            telemetry.addLine("Pos: [" + pipeline.getX() + ", " + pipeline.getY() + "]");
//            telemetry.addData("time", totalTime.milliseconds());
//            telemetry.addLine("RGB");
//            telemetry.addData("Right", Arrays.toString(pipeline.getRight()));
//            telemetry.addData("Left", Arrays.toString(pipeline.getLeft()));
            telemetry.update();
        }
    }

}
