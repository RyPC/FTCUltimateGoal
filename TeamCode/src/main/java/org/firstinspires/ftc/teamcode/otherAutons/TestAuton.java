package org.firstinspires.ftc.teamcode.otherAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="TestAuton", group="Auton")
//@Disabled
public class TestAuton extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    Movement movement = new Movement(this, robotHardware, telemetry);

    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            movement.resetPower();
            movement.turnTo(elapsedTime.seconds() / 2 % 2 == 0 ? 180 : 0);
            movement.setPowers();
        }

    }
}
