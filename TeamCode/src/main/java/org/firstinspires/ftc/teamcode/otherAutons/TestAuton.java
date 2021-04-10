package org.firstinspires.ftc.teamcode.otherAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="TestAuton", group="Auton")
@Disabled
public class TestAuton extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        robotHardware.sleep(1000);
        robotHardware.turnPID(90);
        robotHardware.sleep(1000);
        robotHardware.turnPID(-90);
        robotHardware.sleep(1000);
        robotHardware.turnPID(0);


    }
}
