package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "pooMode", group = "TeleOp")
public class Teleop extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    TeleOpControls teleOpControls = new TeleOpControls(this, robotHardware, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() {
        robotHardware.init(hardwareMap, true);

        while (!opModeIsActive() && !isStopRequested()) {
            idle();
            sleep(50);
        }
        teleOpControls.useCamera();
        robotHardware.wobbleClamp.setPosition(constants.clampOpen);
        while (opModeIsActive() && !isStopRequested()) {
            teleOpControls.autoAim();
            teleOpControls.notDriving();
            teleOpControls.normalDrive();
            telemetry.addData("imu", robotHardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Mode", teleOpControls.getB() ? "low" : "high");
            telemetry.addData("Shooter Vel", robotHardware.shooter.getVelocity());
            telemetry.update();
        }
    }
}
