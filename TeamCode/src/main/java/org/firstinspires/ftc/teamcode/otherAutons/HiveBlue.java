package org.firstinspires.ftc.teamcode.otherAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Hive 3 Ring", group="Auton")
@Disabled
public class HiveBlue extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        robotHardware.strafe(0.25, 500);
        robotHardware.drive(0.5, 1700);
        robotHardware.sleep(1000);
        robotHardware.turnTo(180, 1);
        robotHardware.shooter.setVelocity(constants.shooterPower);
        robotHardware.sleep(2000);
        robotHardware.cleanser.setPower(0.2);
        robotHardware.intake.setPower(0.3);
        robotHardware.sleep(3000);
        robotHardware.shooter.setVelocity(0);
        robotHardware.cleanser.setPower(0);
        robotHardware.intake.setPower(0);
        robotHardware.turnTo(0, 1);
        robotHardware.drive(0.25,500);
        robotHardware.strafe(-0.25, 500);
        robotHardware.turnTo(-90, 1);
        robotHardware.strafe(-0.5, 1500);
        robotHardware.wobbleArm.setPosition(constants.armDown);
        robotHardware.sleep(500);
        robotHardware.wobbleClamp.setPosition(constants.clampClosed);
        robotHardware.strafe(0.5, 125);
        robotHardware.sleep(500);
        robotHardware.wobbleArm.setPosition(constants.armUp);
        robotHardware.sleep(1000);
        robotHardware.turnTo(0, 1);
        robotHardware.strafe(0.5, 800);
        robotHardware.drive(0.5, 500);

    }
}
