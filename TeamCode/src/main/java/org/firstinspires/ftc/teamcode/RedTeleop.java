package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "pooMode", group = "TeleOp")
public class RedTeleop extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    TeleOpControls teleOpControls = new TeleOpControls(this, robotHardware, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);
    OpenCvCamera backboardCamera;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing...");
        telemetry.update();
        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Starting Camera...");
        telemetry.update();

        BackboardPipeline pipeline = new BackboardPipeline(Color.RED);
        backboardCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardware.backboardWebcam);
        backboardCamera.setPipeline(pipeline);

        backboardCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                backboardCamera.startStreaming(constants.cameraWidth, constants.cameraHeight, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        telemetry.addLine("Ready for Start");
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("right", robotHardware.rightEgg.getPosition());
            telemetry.addData("left", robotHardware.leftEgg.getPosition());
            telemetry.update();
            idle();
            sleep(50);
        }

        robotHardware.wobbleClamp.setPosition(constants.clampOpen);
        while (opModeIsActive() && !isStopRequested()) {
            teleOpControls.notDriving();
            teleOpControls.eggs1();
            if (this.gamepad1.y) {
                movement.resetPower();
                if (pipeline.detected() && movement.angleCloseTo(10)) {
                    if (teleOpControls.bPressed)
                        movement.goToPoint(199, 120, pipeline);
                    else
                        movement.goToPoint(133, (int) (124 - (pipeline.getAngle() * 1.25)), pipeline);
                }
                else
                    movement.turnTo(0);
                movement.setPowers();
            }
            else {
                teleOpControls.normalDrive();
                teleOpControls.noCheckBlocker();
            }

            telemetry.addData("imu", robotHardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Mode", teleOpControls.getB() ? "low" : "high");
            telemetry.addData("Shooter Vel", robotHardware.shooter.getVelocity());
            telemetry.update();
        }
    }
}
