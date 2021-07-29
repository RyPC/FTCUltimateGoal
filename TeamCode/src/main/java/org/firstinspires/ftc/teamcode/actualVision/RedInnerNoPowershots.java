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

@Autonomous(name = "Red Inner No Powershots", group = "Vision")
public class RedInnerNoPowershots extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;
    OpenCvCamera ringCamera;
    int cameraMonitorViewId;
    int shooterPower = constants.shooterPower + 20;
    @Override

    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        cameraMonitorViewId = robotHardware.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robotHardware.hwMap.appContext.getPackageName());

        //set up ring camera
        ringCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.ringWebcam);

        Pipeline ringPipeline = new Pipeline(Color.RED, Side.LEFT);
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
            telemetry.addLine("Top RGB: [" + ringPipeline.getRed(Rings.FOUR) + ", " + ringPipeline.getGreen(Rings.FOUR) + ", " + ringPipeline.getBlue(Rings.FOUR) + "]");
            telemetry.addLine("Bot RGB: [" + ringPipeline.getRed(Rings.ONE) + ", " + ringPipeline.getGreen(Rings.ONE) + ", " + ringPipeline.getBlue(Rings.ONE) + "]");
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
                    robotHardware.drivePower(52, 0.5, false, 0, shooterPower);
                    robotHardware.angleAdjustment = 20;
                    stage++;
                    break;
                case 5:
                case 1:
                    //go to shooting position and charge shooter
                    movement.goToPoint(127, 132, pipeline);
                    if (elapsedTime.milliseconds() > 5000) {
                        stage++;
                    }
                    break;
                case 6:
                case 2:
                    //shoot
                    if (robotHardware.blocker.getPosition() != constants.blockerUp)
                        movement.goToPoint(127, 132, pipeline, 0.8);
                    if (movement.closeTo(127, 132, 15, pipeline)) {
                        movement.shoot(shooterPower, 0.35);
                    }
                    if (elapsedTime.milliseconds() > 3250) {
                        stage++;
                        movement.block();
                    }
                    robotHardware.shooter.setVelocity(shooterPower);
                    break;
                case 3:
                    //line up for wobble
                    movement.goToPoint(163, 106, pipeline);
                    if (elapsedTime.milliseconds() > 1000) {
                        stage++;
                        robotHardware.angleAdjustment = 0;
                    }
                    break;
                case 4:
                    //get bouncebacks
                    robotHardware.turnTo(0);
                    robotHardware.drivePower(48, 0.75, false, 0, shooterPower);
                    robotHardware.drivePower(-2, -0.75, false, 0, shooterPower);
                    robotHardware.turnTo(-90);
                    robotHardware.drivePower(34, 0.75, false, -90, shooterPower);
                    //place wobble
                    switch (position) {
                        case ZERO:
                            robotHardware.strafePower(2, 0.75);
                            robotHardware.turnTo(0);
                            robotHardware.drivePower(-35, -0.5, false, 0, shooterPower);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-48, -0.75);
                            break;
                        case ONE:
                            robotHardware.drivePower(-10, 0.75, false, -90, shooterPower);
                            robotHardware.strafePower(4, 0.75);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-4, -0.75);
                            robotHardware.drivePower(-24, -1, false, -90, shooterPower);
                            robotHardware.strafePower(50, 0.75);
                            break;
                        case FOUR:
                            robotHardware.strafePower(2, 0.75);
                            robotHardware.turnTo(0);
                            robotHardware.placeWobble();
                            robotHardware.strafePower(-40, -0.75);
                            robotHardware.drivePower(-40, -1, false, 0, shooterPower);
                            break;
                    }
                    robotHardware.angleAdjustment = 20;
                    robotHardware.turnTo(0, 0.75);
                    stage++;
                    break;
                case 100:
                    //pp-pp-pparkkk
                    robotHardware.angleAdjustment = 20;
                    robotHardware.intakeOff();
                    shooterPower = 0;
                    robotHardware.wobbleClamp.setPosition(constants.clampClosed);
                    robotHardware.wobbleArm.setPosition(constants.armUp);
                    robotHardware.shooter.setPower(0);
//                    if (totalTime.milliseconds() < 25000) {
                    if (false) {

                        int x = (int)(30.0 * Math.cos(totalTime.milliseconds()  * 2 * Math.PI / 3000));
                        int y = (int)(20.0 * Math.sin(totalTime.milliseconds()  * 2 * Math.PI / 3000));

//                        robotHardware.wobbleArm.setPosition((((int)totalTime.seconds()) % 2  == 0) ? constants.armDown : constants.armUp);

                        movement.goToPoint(75 + x, 186 + y, pipeline);

                    }
                    else {
                        movement.goToPoint(165, 91, pipeline, 0.75);
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
