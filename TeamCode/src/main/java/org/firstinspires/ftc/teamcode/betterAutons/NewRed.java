package org.firstinspires.ftc.teamcode.betterAutons;


import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Reb", group = "Auton")
public class NewRed extends LinearOpMode {


    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();


    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        robotHardware.shooter.setVelocity(constants.shooterPower + 100);
        robotHardware.driveTo(20);
        robotHardware.turnTo(-90);
        robotHardware.driveTo(12);
        robotHardware.turnTo(0);
        robotHardware.intake.setPower(0.4);
        ElapsedTime waitTime = new ElapsedTime();
        waitTime.reset();
        while (waitTime.milliseconds() < 5000 && this.opModeIsActive()) {
            if (Math.abs(robotHardware.shooter.getVelocity() - constants.shooterPower + 100) > 50) {
                robotHardware.cleanser.setPower(0);
            }
            else {
                robotHardware.cleanser.setPower(1);
            }
        }
        robotHardware.drive(0.15, 10000);

    }
}
