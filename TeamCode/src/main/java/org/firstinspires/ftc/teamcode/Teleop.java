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

        waitForStart();
        robotHardware.wobbleClamp.setPosition(constants.clampUp);
        while (opModeIsActive() && !isStopRequested()) {
            teleOpControls.allControlsJustToSpiteGiebe();
            telemetry.addData("imu", robotHardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Shooter Vel", robotHardware.shooter.getVelocity());
//            telemetry.addData(r);
            telemetry.update();
        }
    }
}
