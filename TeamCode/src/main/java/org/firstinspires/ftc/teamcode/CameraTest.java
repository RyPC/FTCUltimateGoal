package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Rings;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@TeleOp(name = "camera test", group = "TeleOp")
//@Disabled
public class CameraTest extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    TeleOpControls teleOpControls = new TeleOpControls(this, robotHardware, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        Pipeline pipeline = new Pipeline(Color.BLUE);
        robotHardware.camera.setPipeline(pipeline);

        robotHardware.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotHardware.camera.startStreaming(constants.width, constants.height, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });

        telemetry.addLine("Camera Ready");
        telemetry.update();

        while (!opModeIsActive() && !isStarted() && !isStopRequested()) {
            telemetry.addData("Position", pipeline.getPosition());
            telemetry.addLine("four: [" + pipeline.getRed(Rings.FOUR) + ", " + pipeline.getGreen(Rings.FOUR) + ", " + pipeline.getBlue(Rings.FOUR) + "]");
            telemetry.addLine("one: [" + pipeline.getRed(Rings.ONE) + ", " + pipeline.getGreen(Rings.ONE) + ", " + pipeline.getBlue(Rings.ONE) + "]");
            telemetry.update();
        }


    }
}
