package org.firstinspires.ftc.teamcode.otherAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

//blue inner
//three high goal
@Autonomous(name="Blue inner/high goal", group="Auton")
//@Disabled
public class NuclearMindsBlue extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        robotHardware.shooter.setVelocity(constants.shooterPower);
        robotHardware.sleep(500);
        robotHardware.strafeTo(12);
        robotHardware.sleep(500);
        robotHardware.driveTo(57, false, 0);
        robotHardware.sleep(500);
        robotHardware.turnTo(10, 1);
        robotHardware.shoot(10000, constants.shooterPower);
        robotHardware.turnTo(0, 1);
        robotHardware.driveTo(8, false, 0);
        robotHardware.strafeTo(8);


    }
}
