package org.firstinspires.ftc.teamcode.timeAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Reb Bowel", group="Auton")
@Disabled
public class RebAubtob extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        //start shooter
        robotHardware.shooter.setVelocity(constants.shooterPower);
        robotHardware.sleep(250);
        //drive to shoot pos
        robotHardware.drive(0.25, 1500);
        robotHardware.strafe(-0.25, 2000);
        robotHardware.sleep(500);
        //shoot power shots
        //1
        robotHardware.turnTo(-9, 1);
        robotHardware.sleep(1500);
        robotHardware.cleanser.setPower(0.25);
        robotHardware.intake.setPower(0.4);
        robotHardware.shooter.setVelocity(1350);
        robotHardware.sleep(750);
        robotHardware.cleanser.setPower(0);
        robotHardware.intake.setPower(0);
        robotHardware.sleep(500);
        //2
        robotHardware.shooter.setVelocity(1450);
        robotHardware.turnTo(-7, 1);
        robotHardware.cleanser.setPower(0.25);
        robotHardware.intake.setPower(0.4);
        robotHardware.sleep(700);
        robotHardware.cleanser.setPower(0);
        robotHardware.intake.setPower(0);
        robotHardware.sleep(500);
        //3
        robotHardware.turnTo(-5, 1);
        robotHardware.cleanser.setPower(0.25);
        robotHardware.intake.setPower(0.4);
        robotHardware.sleep(1200);
        robotHardware.strafe(0.25,250);
        //last 0-4 rings
        //reverse intake
        robotHardware.shooter.setVelocity(constants.shooterPower);
        robotHardware.intake.setPower(-0.25);
        robotHardware.drive(0.15, 2000);
        robotHardware.intake.setPower(0.35);
        robotHardware.sleep(500);
        robotHardware.turnTo(5, 1);
        for(int i = 0; i < 6; i++) {
            robotHardware.drive(0.1, 1000);
            robotHardware.intake.setPower(-0.35);
            robotHardware.drive(0.1, 125);
            robotHardware.intake.setPower(0.35);
        }
        robotHardware.sleep(1000);
        robotHardware.shooter.setVelocity(0);
        robotHardware.cleanser.setPower(0);
        robotHardware.drive(0.5, 750);
        robotHardware.turnTo(-90, 1);
        robotHardware.wobbleArm.setPosition(constants.armDown);
        robotHardware.wobbleClamp.setPosition(constants.clampClosed);
        robotHardware.wobbleArm.setPosition(constants.armUp);
        robotHardware.strafe(-1, 250);
        robotHardware.turnTo(0, 1);

    }
}
