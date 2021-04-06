package org.firstinspires.ftc.teamcode.otherAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Blocking Blue", group="Auton")
public class BlockingBlue extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        robotHardware.driveTo(8);
        robotHardware.driveTo(68, -30);
        robotHardware.turnTo(45);

    }
}
