package org.firstinspires.ftc.teamcode.otherAutons;


import android.os.DropBoxManager;
//import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Red Out", group = "Auton")
@Disabled
public class RedOuter extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();


    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);
        robotHardware.wobbleClamp.setPosition(constants.clampClosed);

        waitForStart();

        //start shooter and drive in front of rings
        robotHardware.shooter.setVelocity(constants.shooterPower + 50);
        robotHardware.driveTo(20, false, 0);
        robotHardware.sleep(250);
        robotHardware.strafeTo(-12);
        //shoot initial 3 rings
        robotHardware.sleep(500);
        robotHardware.turnTo(-6, 2);
        robotHardware.shoot(750, constants.shooterPower + 50);
        robotHardware.shoot(3250, constants.shooterPower);
        //knock over middle rings
        robotHardware.intake.setPower(-0.25);
        robotHardware.driveTo(6, false,-5);
        robotHardware.sleep(750);
        robotHardware.intake.setPower(0.5);
        //intake middle rings while shooting
        robotHardware.sleep(1000);
        for (int i = 0; i < 5; i++) {
            robotHardware.drivePower(5, 0.1, true, -4, constants.shooterPower - 60);
            robotHardware.intake.setPower(-0.15);
            robotHardware.drivePower(3, 0.1, false, -4, 0);
            robotHardware.intake.setPower(0.75);
        }
        robotHardware.shoot(3000, constants.shooterPower);
        robotHardware.driveTo(12, false,0);
        robotHardware.shooter.setVelocity(0);
        robotHardware.cleanser.setPower(0);
        robotHardware.intake.setPower(0);
        //place wobble
        robotHardware.turnTo(90, 0.35);
        robotHardware.strafeTo(8);
        robotHardware.wobbleArm.setPosition(constants.armDown);
        robotHardware.sleep(500);
        robotHardware.wobbleClamp.setPosition(constants.clampOpen);
        robotHardware.sleep(500);
        robotHardware.wobbleArm.setPosition(constants.armUp);
        robotHardware.sleep(500);
        robotHardware.strafeTo(-8);
        robotHardware.turnTo(0, 0.5);

    }
}
