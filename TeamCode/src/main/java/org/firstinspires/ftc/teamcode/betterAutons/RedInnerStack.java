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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Inner + Stack", group = "Auton")
@Disabled
public class RedInnerStack extends LinearOpMode {
    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    RedInner redInner;
    Rings position;


    @Override
    public void runOpMode() {
        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Setting up camera...");
        telemetry.update();

        Pipeline pipeline = new Pipeline(Color.RED);
        robotHardware.backboardCamera.setPipeline(pipeline);

        robotHardware.backboardCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotHardware.backboardCamera.startStreaming(constants.cameraWidth, constants.cameraHeight, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });

        //camera stuff
        while (!isStarted() && !isStopRequested()) {
            position = pipeline.getPosition();
            telemetry.addData("Position", position);
            telemetry.addLine("four: [" + pipeline.getRed(Rings.FOUR) + ", " + pipeline.getGreen(Rings.FOUR) + ", " + pipeline.getBlue(Rings.FOUR) + "]");
            telemetry.addLine("one: [" + pipeline.getRed(Rings.ONE) + ", " + pipeline.getGreen(Rings.ONE) + ", " + pipeline.getBlue(Rings.ONE) + "]");
            telemetry.update();
            idle();
            sleep(50);
        }
        robotHardware.backboardCamera.stopStreaming();

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        //start moving and stuff
        robotHardware.turnTo(0, 0.5);

        //start
        robotHardware.shooter.setVelocity(constants.shooterPower - 50);
        robotHardware.sleep(500);
        robotHardware.strafeTo(-12);
        robotHardware.sleep(500);
        robotHardware.driveTo(57);
        robotHardware.sleep(500);
        robotHardware.turnTo(-17, 1);
        robotHardware.shoot(5000, constants.shooterPower - 50);
        robotHardware.shooter.setVelocity(0);
        robotHardware.turnTo(0, 1);
        robotHardware.driveTo(8);

        //wobble goal
        switch(position) {
            case ZERO:
                robotHardware.strafeTo(24);
                robotHardware.placeWobble();
                robotHardware.strafeTo(-24);
                break;
            case ONE:
                robotHardware.driveTo(24);
                robotHardware.strafeTo(6);
                robotHardware.placeWobble();
                robotHardware.strafeTo(-6);
                robotHardware.driveTo(-24);
                break;
            case FOUR:
                robotHardware.driveTo(48);
                robotHardware.strafeTo(24);
                robotHardware.placeWobble();
                robotHardware.strafeTo(-24);
                robotHardware.driveTo(-48);
                break;
        }


        //go back for more rings
        if (elapsedTime.seconds() < 20 && position != Rings.ZERO) {
            robotHardware.shooter.setVelocity(constants.shooterPower);
            robotHardware.strafeTo(24);
            robotHardware.turnTo(180);
            robotHardware.intakeOn();
            robotHardware.drivePower(12, 0.25, false, 180, 0);
            robotHardware.driveTo(-6, 180);
            robotHardware.intakeOff();
            robotHardware.turnTo(-3);
            robotHardware.shoot(5000, constants.shooterPower);
            robotHardware.shooter.setVelocity(0);
            robotHardware.turnTo(0);
            robotHardware.driveTo(12);
        }

    }
}
