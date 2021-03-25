package org.firstinspires.ftc.teamcode.actualAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Reb Bowel No Power", group="Auton")
@Disabled
public class RebAubtobNoPower extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        //start shooter
        robotHardware.shooter.setVelocity(constants.shooterPower);
        robotHardware.sleep(250);
        robotHardware.drive(0.25, 1750);
        robotHardware.strafe(-0.25, 1750);
        robotHardware.sleep(500);
        robotHardware.turnTo(5);
        robotHardware.sleep(500);
        for (int i = 0; i < 4; i++) {
            robotHardware.cleanser.setPower(0.25);
            robotHardware.intake.setPower(0.4);
            robotHardware.sleep(500);
            robotHardware.cleanser.setPower(0);
            robotHardware.intake.setPower(0);
            robotHardware.sleep(500);
            if (i == 0) {
                robotHardware.shooter.setVelocity(constants.shooterPower - 50);
            }
        }
        robotHardware.shooter.setVelocity(constants.shooterPower - 50);
        robotHardware.cleanser.setPower(0.25);
        robotHardware.intake.setPower(0.4);
        robotHardware.turnTo(5);
        robotHardware.intake.setPower(-0.2);
        robotHardware.drive(0.15, 500);
        robotHardware.strafe(0.15, 200);
        for(int i = 0; i < 8; i++) {
            robotHardware.drive(0.1, 1000);
            robotHardware.intake.setPower(-0.35);
            robotHardware.drive(0.1, 125);
            robotHardware.intake.setPower(0.35);
        }
        robotHardware.sleep(1000);
        robotHardware.drive(0.25, 500);
        robotHardware.strafe(0.5, 500);
        robotHardware.sleep(2000);
        robotHardware.drive(0.25, 1000);


    }
}
