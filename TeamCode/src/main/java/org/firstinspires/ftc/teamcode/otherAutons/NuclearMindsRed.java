package org.firstinspires.ftc.teamcode.otherAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

//Red Inner
//High goal guess wobble
@Autonomous(name="Red Inner High", group="Auton")
//@Disabled
public class NuclearMindsRed extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        robotHardware.shooter.setVelocity(constants.shooterPower - 50);
        robotHardware.sleep(500);
        robotHardware.strafeTo(-12);
        robotHardware.sleep(500);
        robotHardware.driveTo(57, false, 0);
        robotHardware.sleep(500);
        robotHardware.turnTo(-20, 1);
        robotHardware.shoot(10000, constants.shooterPower - 50);
        robotHardware.shooter.setVelocity(0);
        robotHardware.turnTo(0, 1);
        robotHardware.driveTo(24, false, 0);
        robotHardware.strafeTo( 7);
        robotHardware.wobbleArm.setPosition(constants.armDown);
        robotHardware.sleep(500);
        robotHardware.wobbleClamp.setPosition(constants.clampOpen);
        robotHardware.sleep(1000);
        robotHardware.wobbleArm.setPosition(constants.armUp);
        robotHardware.sleep(1000);
        robotHardware.strafeTo(-7);
        robotHardware.turnTo(45, 1);
        robotHardware.strafeTo(-16);
        robotHardware.turnTo(0,1);

    }
}
