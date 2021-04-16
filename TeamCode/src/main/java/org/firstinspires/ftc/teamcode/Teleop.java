package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "pooMode", group = "TeleOp")
public class Teleop extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    TeleOpControls teleOpControls = new TeleOpControls(this, robotHardware, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing...");
        telemetry.update();
        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Starting Camera...");
        telemetry.update();

        BackboardPipeline pipeline = new BackboardPipeline(Color.RED);
        robotHardware.camera.setPipeline(pipeline);

        robotHardware.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotHardware.camera.startStreaming(constants.width, constants.height, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        telemetry.addLine("Ready for Start");
        while (!opModeIsActive() && !isStopRequested()) {
            idle();
            sleep(50);
        }

        robotHardware.wobbleClamp.setPosition(constants.clampOpen);
        while (opModeIsActive() && !isStopRequested()) {
            if (this.gamepad1.y) {
                movement.resetPower();
                movement.turnToX(135, pipeline);
                movement.setPowers();
            }
            else {
                teleOpControls.notDriving();
                teleOpControls.normalDrive();
            }
            telemetry.addData("imu", robotHardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Mode", teleOpControls.getB() ? "low" : "high");
            telemetry.addData("Shooter Vel", robotHardware.shooter.getVelocity());
            telemetry.update();
        }
    }
}
