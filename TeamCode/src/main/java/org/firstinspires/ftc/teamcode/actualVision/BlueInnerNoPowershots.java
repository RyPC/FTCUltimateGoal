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

@Autonomous(name = "Blue Inner No Powershots", group = "Vision")
public class BlueInnerNoPowershots extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;
    OpenCvCamera ringCamera;
    int cameraMonitorViewId;
    int shooterPower;
    final int lowShooterPower = constants.shooterPower - 190;
    final int highShooterPower = constants.shooterPower + 100;
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

        shooterPower = highShooterPower;
        robotHardware.shooter.setVelocity(shooterPower);
        while(opModeIsActive() && !isStopRequested()) {
            movement.resetPower();

            if (stage != prevStage) {
                elapsedTime.reset();
                prevStage = (int)stage;
            }

            switch (stage) {
                case 0:
                    robotHardware.drivePower(48, 0.5, false, 0, shooterPower);
                    robotHardware.strafePower(6, 0.5);
                    robotHardware.angleAdjustment = -12;
                    stage++;
                    break;
                case 6:
                case 1:
                    robotHardware.angleAdjustment = -12;
                    movement.goToPoint(106, 144, pipeline);
                    if (elapsedTime.milliseconds() > (stage == 1 ? 5000 : 2000)) {
                        stage++;
                    }
                    break;
                case 2:
                case 7:
                    //move to shooter position(145, 130)
                    //shoot rings into high goal
                    robotHardware.angleAdjustment = -12;
                    if (robotHardware.blocker.getPosition() != constants.blockerUp) {
                        movement.goToPoint(106, 144, pipeline);
                        robotHardware.intakeOff();
                    }
                    if (movement.closeTo(106, 144, 15, pipeline) && robotHardware.getPowers() < 1.25 && movement.angleCloseTo(2.5)) {
                        movement.shoot(shooterPower, 0.35);
                    }
                    if (elapsedTime.milliseconds() > 3000) {
                        stage++;
                        shooterPower = constants.shooterPower - 150;
                        movement.block();
                        robotHardware.angleAdjustment = -8;
                    }
                    robotHardware.shooter.setVelocity(constants.shooterPower);
                    break;
                case 3:
                    movement.goToPoint(position == Rings.ONE ? 130 : 135, 90, pipeline);
                    if (elapsedTime.milliseconds() > 2500) {
                        stage++;
                        robotHardware.angleAdjustment = 0;
                    }
                    break;
                case 4:
                    //deposit wobble goal
                    robotHardware.shooter.setVelocity(shooterPower);
                    switch (position) {
                        case ZERO:
                            robotHardware.turnTo(0);
                            robotHardware.strafePower(6, 0.75);
                            robotHardware.drivePower(52, 0.33, false, 0, shooterPower);
                            robotHardware.drivePower(-24, -0.33, false, 0, shooterPower);
                            robotHardware.turnTo(-90);
                            robotHardware.drivePower(-35, -0.5, false, -90, shooterPower);
                            robotHardware.strafePower(13, 0.75);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-8, -0.75);
                            robotHardware.drivePower(40, 1, false, -90, shooterPower);
                            robotHardware.turnTo(0);
                            robotHardware.drivePower(-38, -1, false, 0, shooterPower);
                            break;
                        case ONE:
                            robotHardware.turnTo(0);
                            robotHardware.strafePower(6, 0.75);
                            robotHardware.drivePower(52, 0.33, false, 0, shooterPower);
                            robotHardware.drivePower(-24, -0.33, false, 0, shooterPower);
                            robotHardware.turnTo(180, 0.5);
                            robotHardware.strafePower(6, 0.5);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-6, -0.5);
                            robotHardware.strafePower(-4, -0.5);
                            robotHardware.turnTo(0);
                            robotHardware.drivePower(-20, -0.5, false, 0, shooterPower);
                            break;
                        case FOUR:
                            robotHardware.turnTo(30);
                            robotHardware.wobbleArm.setPosition(constants.armDown);
                            robotHardware.drivePower(42, 0.75, false, 45, shooterPower);
                            robotHardware.turnTo(90);
                            robotHardware.wobbleClamp.setPosition(constants.clampOpen);
                            robotHardware.strafePower(-5, -0.5);
                            robotHardware.drivePower(-12, -0.75, false, 90, shooterPower);
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
                    if (position == Rings.FOUR) {
                        robotHardware.drivePower(13, 0.5, false, 0, shooterPower);
                        robotHardware.turnTo(-90);
                        robotHardware.drivePower(28, 0.5, false, -90, shooterPower);
                        robotHardware.strafePower(8, 1);
                        robotHardware.turnTo(-55);
                        robotHardware.drivePower(4, 0.75, false, -55, shooterPower);
                        robotHardware.turnTo(0);
                        robotHardware.turnTo(-20);
                        robotHardware.drivePower( -60, -0.75, false, -20, shooterPower);
                        robotHardware.turnTo(0);
                    }
                    robotHardware.intakeOff();
                    robotHardware.turnTo(12);
                    stage++;
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
                        robotHardware.angleAdjustment = -14;
                        movement.goToPoint(111, 96, pipeline);
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
