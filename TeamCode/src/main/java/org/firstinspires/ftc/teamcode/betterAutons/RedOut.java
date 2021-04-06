package org.firstinspires.ftc.teamcode.betterAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.firstinspires.ftc.teamcode.enums.Stages;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Out", group = "Auton")
@Disabled
public class RedOut extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Stages stage;
    Rings position;

    @Override
    public void runOpMode() throws InterruptedException {
        stage = Stages.INIT;
        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        Pipeline pipeline = new Pipeline(Color.BLUE);
        robotHardware.camera.setPipeline(pipeline);

        robotHardware.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotHardware.camera.startStreaming(constants.width, constants.height, OpenCvCameraRotation.UPRIGHT);
            }
        });

        //camera stuff
        stage = Stages.CAMERA;
        while (!isStarted() && !isStopRequested() && !opModeIsActive()) {
            position = pipeline.getPosition();
            telemetry.addData("Position", position);
            telemetry.addLine("four: [" + pipeline.getRed(Rings.FOUR) + ", " + pipeline.getGreen(Rings.FOUR) + ", " + pipeline.getBlue(Rings.FOUR) + "]");
            telemetry.addLine("one: [" + pipeline.getRed(Rings.ONE) + ", " + pipeline.getGreen(Rings.ONE) + ", " + pipeline.getBlue(Rings.ONE) + "]");
            telemetry.update();
        }
        robotHardware.camera.stopStreaming();
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        //start moving and stuff
        stage = (position == Rings.ZERO) ? Stages.ZERO_TO_SHOOT : Stages.TO_RINGS;
        robotHardware.turnTo(0, 1.5);
        while (!isStopRequested() && opModeIsActive()) {

            switch (stage) {

                case ZERO_TO_SHOOT:
                    robotHardware.driveTo(62);
                    stage = Stages.SHOOT_FROM_WALL;
                    break;

                case SHOOT_FROM_WALL:
                    robotHardware.strafeTo(-8);
                    robotHardware.shoot(7500);
                    robotHardware.strafeTo(-8);
                    stage = Stages.WOBBLE_FROM_LINE;
                    break;

                case SHOOT_FROM_LINE:
                    robotHardware.shoot(10000);
                    robotHardware.shooter.setVelocity(0);
                    robotHardware.driveTo(12);
                    stage = Stages.WOBBLE_FROM_LINE;
                    break;

                case PARK_FROM_LINE:
                    robotHardware.turnTo(0, 3);
                    robotHardware.driveTo(8);
                    stage = Stages.REST;
                    break;

                case TO_RINGS:
                    robotHardware.driveTo(16, false, 0);
                    robotHardware.strafeTo(-12);
                    stage = Stages.SHOOT_FROM_RINGS;
                    break;

                case WOBBLE_FROM_LINE:
                    switch (position) {
                        case ZERO:
                            robotHardware.strafeTo(12);
                            robotHardware.turnTo(180, 0.25);
                            robotHardware.placeWobble();
                            robotHardware.strafeTo(12);
                            robotHardware.turnTo(0, 0.25);
                            robotHardware.strafeTo(-24);
                            stage = Stages.REST;
                            break;
                        case ONE:
                            robotHardware.driveTo(24);
                            robotHardware.strafeTo(-6);
                            robotHardware.turnTo(180, 0.25);
                            robotHardware.placeWobble();
                            robotHardware.strafeTo(6);
                            robotHardware.turnTo(0, 0.25);
                            robotHardware.driveTo(-24);
                            stage = Stages.REST;
                            break;
                        case FOUR:
                            robotHardware.driveTo(58);
                            robotHardware.strafeTo(36);
                            robotHardware.turnTo(180, 0.15);
                            robotHardware.placeWobble();
                            robotHardware.strafeTo(12);
                            robotHardware.turnTo(0, 0.15);
                            robotHardware.strafeTo(-24);
                            robotHardware.driveTo(-48);
                            stage = Stages.REST;
                            break;
                    }
                    break;

                case REST:
                    idle();
                    break;

                case SHOOT_FROM_RINGS:
                    robotHardware.cleanser.setPower(-0.5);
                    robotHardware.intake.setPower(-1);
                    robotHardware.sleep(250);
                    robotHardware.cleanser.setPower(0);
                    robotHardware.intake.setPower(0);
                    robotHardware.turnTo(-3, 2);
                    robotHardware.shoot(4000, constants.shooterPower + 50);
                    robotHardware.shooter.setVelocity(0);
                    robotHardware.turnTo(0, 2);
                    stage = (elapsedTime.milliseconds() < 12500) ? Stages.RINGS : Stages.RINGS_TO_LINE;
                    break;

                case RINGS_TO_LINE:
                    robotHardware.driveTo(48);
                    robotHardware.strafeTo(8);
                    stage = Stages.WOBBLE_FROM_LINE;
                    break;

                case RINGS:
                    robotHardware.shooter.setVelocity(0);
                    robotHardware.intake.setPower(0.75);
                    robotHardware.cleanser.setPower(0.5);
                    robotHardware.drivePower(18, 0.33, false, 0, 0);
                    robotHardware.drivePower(-18, -0.33, false, 0, 0);
                    stage = Stages.SHOOT_FROM_RINGS;
                    break;


                default:
                    robotHardware.shooter.setVelocity(0);
                    stage = Stages.REST;
                    break;

            }

            telemetry.addData("Stage", stage);
            telemetry.addData("Position", position);
            telemetry.update();

        }

    }
}
