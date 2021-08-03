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

@Autonomous(name = "New Red Outer", group = "Vision")
public class RedOuterNew extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;
    OpenCvCamera ringCamera;
    int cameraMonitorViewId;
    int shooterPower = constants.shooterPower - 40;
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

            if (stage != prevStage) {
                movement.ix = 0;
                movement.iy = 0;
                elapsedTime.reset();
                prevStage = (int)stage;
            }

            switch (stage) {
                case 0:
                    //wobble goal
                    robotHardware.shooter.setVelocity(shooterPower);
                    switch(position) {
                        case ZERO:
                            robotHardware.drivePower(68, 1, false, 0, shooterPower);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-11, -1);
                            robotHardware.drivePower(-10, -1, false, 0, shooterPower);
                            break;
                        case ONE:
                            robotHardware.drivePower(86, 1, false, 0, shooterPower);
                            robotHardware.turnTo(180);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-5, -1);
                            robotHardware.turnTo(0);
                            robotHardware.drivePower(-40, -0.75, false, 0, shooterPower);
                            robotHardware.sleep(500);
                            break;
                        case FOUR:
                            robotHardware.shooter.setVelocity(shooterPower);
                            robotHardware.wobbleArm.setPosition(constants.armDown);
                            robotHardware.drivePower(125, 1, false, 0, shooterPower);
                            robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                            robotHardware.strafePower(-7, -1);
                            robotHardware.drivePower(-48, -0.9, false, 0, shooterPower);
                            robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                            robotHardware.wobbleArm.setPosition(constants.armUp);
                            robotHardware.sleep(250);
                            break;
                    }
                    stage++;
                    break;
                case 1:
                    movement.goToPoint(135, 124, pipeline, 1.25);
                    if (elapsedTime.milliseconds() > (position == Rings.ZERO ? 12000 : 1500)) {
                        stage++;
                    }
                    break;
                case 2:
                    //shoot
                    if (robotHardware.blocker.getPosition() != constants.blockerUp)
                        movement.goToPoint(135, 124, pipeline, 0.8);
                    if (movement.closeTo(135, 124, 15, pipeline)) {
                        movement.shoot(shooterPower, 0.35);
                    }
                    if (elapsedTime.milliseconds() > 3000) {
                        if (position == Rings.ZERO)
                            stage = 100;
                        else
                            stage++;
                        movement.block();
                        shooterPower = constants.shooterPower - 10;
                        robotHardware.shooter.setVelocity(shooterPower);

                    }
                    break;
                case 3:
                    movement.goToPoint(72, 129, pipeline);
                    if (elapsedTime.milliseconds() > 333) {
                        stage++;
                    }
                    break;
                case 4:
                    movement.goToPoint(87, 204, pipeline);
                    if (elapsedTime.milliseconds() > 750) {
                        stage++;
                    }
                    break;
                case 5:
                    movement.goToPoint(168, 196, pipeline);
                    if (elapsedTime.milliseconds() > 3000) {
                        robotHardware.turnTo(-10);
                        stage++;
                    }
                    break;
                case 6:
                    //collect stack while shooting
                    robotHardware.intakeOn();
                    movement.block();
                    switch (position) {
                        case FOUR:
                            robotHardware.drivePower(12, 0.125, false, -10, shooterPower);
                            robotHardware.intake(-0.5);
                            robotHardware.drivePower(-6, -0.2, false, -10, shooterPower);
                            robotHardware.intakeOn();
                            robotHardware.drivePower(-6, -0.2, false, -10, shooterPower);
                            robotHardware.sleep(2500);
                            robotHardware.blocker.setPosition(constants.blockerUp);
                            robotHardware.cleanser.setPower(0.35);
                            robotHardware.sleep(2000);
                            robotHardware.blocker.setPosition(constants.blockerDown);
                            shooterPower = constants.shooterPower;
                            robotHardware.shooter.setVelocity(shooterPower);
                            robotHardware.drivePower(18, 0.25, false, -10, shooterPower);
                            robotHardware.sleep(2000);
    //                    robotHardware.turnTo(-4, 0.5);
                            robotHardware.blocker.setPosition(constants.blockerUp);
                            robotHardware.sleep(1500);
                            break;
                        case ONE:
                            shooterPower = constants.shooterPower;
                            robotHardware.shooter.setVelocity(shooterPower);
                            robotHardware.drivePower(18, 0.25, false, -10, shooterPower);
                            robotHardware.sleep(4000);
                            robotHardware.blocker.setPosition(constants.blockerUp);
                            robotHardware.sleep(1500);
                            break;
                    }
                    stage++;
                    break;
                case 100:
                    //pp-pp-pparkkk
                    robotHardware.angleAdjustment = 0;
                    robotHardware.shooter.setVelocity(0);
                    robotHardware.intakeOff();
                    if (totalTime.milliseconds() < 27000) {

                        int x = (int) (Math.round((30.0 * Math.cos(totalTime.milliseconds()  * 2 * Math.PI / 3000)) / 10.0) * 10);
                        int y = (int) (Math.round((30.0 * Math.sin(totalTime.milliseconds()  * 2 * Math.PI / 3000)) / 10.0) * 10);

                        movement.goToPoint(130 + x, 189 + y, pipeline);
                    }
                    else {
                        movement.goToPoint(153, 90, pipeline, 0.5);
                    }
                    break;
                default:
                    robotHardware.shooter.setPower(0);
                    stage++;
                    break;
            }

            if (totalTime.milliseconds() > 27500)
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
