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

@Autonomous(name = "Blue Inner", group = "Vision")
public class BlueInner extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;
    OpenCvCamera ringCamera;
    int cameraMonitorViewId;
    int shooterPower;
    final int lowShooterPower = constants.shooterPower - 190;
    final int highShooterPower = constants.shooterPower + 45;
    @Override

    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        cameraMonitorViewId = robotHardware.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robotHardware.hwMap.appContext.getPackageName());

        //set up ring camera
        ringCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.ringWebcam);

        Pipeline ringPipeline = new Pipeline(Color.BLUE, Side.RIGHT);
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

        shooterPower = lowShooterPower;
        robotHardware.shooter.setVelocity(shooterPower);
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();

            if (stage != prevStage) {
                elapsedTime.reset();
                prevStage = (int)stage;
            }

            switch (stage) {
                case 0:
                    robotHardware.drivePower(48, 0.75);
                    robotHardware.strafePower(6, 0.75);
                    stage++;
                    break;
                case 1:
                    //move to shooter position(100, 120)
                    //wait to charge shooter?
                    robotHardware.shooter.setVelocity(shooterPower);
                    movement.goToPoint(100, 135, pipeline);
                    if (elapsedTime.milliseconds() > 5000) {
                        stage++;
                    }
                    break;
                case 2:
                    //shoot initial three rings at below angles
                    shooterPower = lowShooterPower;
                    int[] angles = {-8, -12, -17};
                    int[] times = {200, 300, 600};
                    robotHardware.intakeOff();
                    robotHardware.blocker.setPosition(constants.blockerUp);
                    for (int i = 0; i < 3; i++) {
                        shooterPower = lowShooterPower;
                        robotHardware.shooter.setVelocity(shooterPower);
                        robotHardware.turnTo(angles[i], 0.15);
                        robotHardware.intake(0.5);
                        robotHardware.sleep(times[i]);
                        robotHardware.intakeOff();
                    }
                    robotHardware.turnTo(0);
                    stage++;
                    break;
                case 3:
                    movement.goToPoint(135, 90, pipeline);
                    if (elapsedTime.milliseconds() > 2000) {
                        stage++;
                    }
                    break;
                case 4:
                    //deposit wobble goal
                    shooterPower = highShooterPower;
                    robotHardware.shooter.setVelocity(shooterPower);
                    switch (position) {
                        case ZERO:
                            robotHardware.turnTo(180);
                            robotHardware.strafePower(20, 0.75);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-10, -0.75);
                            robotHardware.turnTo(0);
                            break;
                        case ONE:
                            robotHardware.turnTo(45);
                            robotHardware.drivePower(5, 0.33);
                            robotHardware.turnTo(100);
                            robotHardware.placeWobble();
                            robotHardware.turnTo(90);
                            robotHardware.drivePower(-20, -0.5, 90);
                            robotHardware.turnTo(0);
                            break;
                        case FOUR:
                            robotHardware.turnTo(30);
                            robotHardware.wobbleArm.setPosition(constants.armDown);
                            robotHardware.drivePower(38, 0.75, 45);
                            robotHardware.turnTo(120);
                            robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                            robotHardware.strafePower(-17, -0.5);
                            robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                            robotHardware.wobbleArm.setPosition(constants.armUp);
                            robotHardware.turnTo(0);
                            break;
                    }
                    stage++;
                    break;
                case 5:
                    //collect bouncebacks
                    robotHardware.intakeOn();
                    robotHardware.turnTo(0);
                    switch (position) {
                        case ZERO:
                            robotHardware.drivePower(38, 0.5);
                            robotHardware.turnTo(-90);
                            robotHardware.drivePower(48, 0.5, -90);
                            robotHardware.strafePower(5, 1);
                            robotHardware.turnTo(-55);
                            robotHardware.drivePower(6, 0.75);
                            robotHardware.turnTo(0);
                            robotHardware.turnTo(-20);
                            robotHardware.drivePower( -60, -0.75, -20);
                            break;
                        case ONE:
                            robotHardware.drivePower(30, 0.25);
                            break;
                        case FOUR:
                            robotHardware.drivePower(18, 0.5);
                            robotHardware.turnTo(-90);
                            robotHardware.drivePower(48, 0.5, -90);
                            robotHardware.strafePower(5, 1);
                            robotHardware.turnTo(-55);
                            robotHardware.drivePower(4, 0.75, -55);
                            robotHardware.turnTo(0);
                            robotHardware.turnTo(-20);
                            robotHardware.drivePower( -60, -0.75, -20);
                            robotHardware.turnTo(0);
                            break;
                    }
                    robotHardware.intakeOff();
                    stage++;
                    break;
                case 6:
                    //move to shooter position(135, 120)
                    //shoot (up to) three rings into high goal(0 degrees/straight)
                    robotHardware.angleAdjustment = -10;
                    if (robotHardware.blocker.getPosition() != constants.blockerUp)
                        movement.goToPoint(154, 130, pipeline);
                    if (movement.closeTo(154, 130, 15, pipeline) && robotHardware.getPowers() < 1.25 && movement.angleCloseTo(2.5)) {
                        movement.shoot();
                        robotHardware.intakeOn();
                    }
                    if (elapsedTime.milliseconds() > 3000) {
                        stage++;
                        movement.block();
                    }
                    robotHardware.shooter.setVelocity(constants.shooterPower);
                    break;
                case 100:
                    //wait back?
                    //park on line
                        //farther right if in A position
                    robotHardware.intakeOff();
                    shooterPower = 0;
                    robotHardware.shooter.setVelocity(shooterPower);
                    if (totalTime.milliseconds() < 25000) {
//                        movement.goToPoint());
                    }
                    else {
                        movement.goToPoint(153, 90, pipeline);
                    }
                default:
                    stage = 100;
                    break;
            }

            if (totalTime.milliseconds() > 28500)
                stage = 100;
            movement.setPowers();
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
