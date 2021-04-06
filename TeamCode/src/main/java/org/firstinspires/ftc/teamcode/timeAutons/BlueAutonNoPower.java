package org.firstinspires.ftc.teamcode.timeAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Blue No Power", group = "Auton")
@Disabled
public class BlueAutonNoPower extends LinearOpMode {


    RobotHardware robotHardware = new RobotHardware(this, telemetry);


    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        //start shooter
        robotHardware.strafe(0.25, 350);
        robotHardware.shooter.setVelocity(1525);
        robotHardware.sleep(250);
        //drive to shoot pos
        robotHardware.drive(0.25, 1900);
        robotHardware.sleep(500);
        //shoot 3 rings
        robotHardware.cleanser.setPower(0.25);
        robotHardware.intake.setPower(0.20);
        robotHardware.shooter.setVelocity(1450);
        robotHardware.sleep(500);
        robotHardware.cleanser.setPower(0);
        robotHardware.intake.setPower(0);
        robotHardware.sleep(500);
        robotHardware.cleanser.setPower(0.25);
        robotHardware.intake.setPower(0.20);
        robotHardware.sleep(2500);
        robotHardware.intake.setPower(-0.25);
        robotHardware.drive(0.125, 2000);
        robotHardware.intake.setPower(0.35);
        robotHardware.sleep(500);
        for (int i = 0; i < 6; i++) {
            robotHardware.drive(0.15, 1000);
            robotHardware.intake.setPower(-0.35);
            robotHardware.drive(0.15, 100);
            robotHardware.intake.setPower(0.35);
        }
        robotHardware.sleep(2000);
        robotHardware.shooter.setPower(0);
        robotHardware.cleanser.setPower(0);
        robotHardware.drive(0.5, 500);

    }
}
