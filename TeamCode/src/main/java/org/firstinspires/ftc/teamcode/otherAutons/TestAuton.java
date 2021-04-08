package org.firstinspires.ftc.teamcode.otherAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="TestAuton", group="Auton")
//@Disabled
public class TestAuton extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        robotHardware.shooter.setVelocity(constants.shooterPower);
        robotHardware.sleep(5000);
        robotHardware.turnTo(2,  1.5);
        robotHardware.turnTo(-2,  1.5);
        robotHardware.turnTo(0,  1.5);
        robotHardware.shootRings();


    }
}
