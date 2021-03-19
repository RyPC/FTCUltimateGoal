package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Blue Autonomous", group="Auton")

public class blueAuton extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);


    @Override
    public void runOpMode() {

        robotHardware.init(hardwareMap, true);

        waitForStart();

        //start shooter
        robotHardware.shooter.setVelocity(1550);
        robotHardware.sleep(2000);
        //drive and shoot first 3 rings
        robotHardware.drive(0.25, 2000);
        robotHardware.sleep(500);
        robotHardware.cleanser.setPower(0.20);
        robotHardware.intake.setPower(0.20);
        robotHardware.shooter.setVelocity(1500);
        robotHardware.sleep(3000);
        robotHardware.intake.setPower(0.35);
        robotHardware.drive(0.125, 7000);
        robotHardware.sleep(2000);
        robotHardware.shooter.setPower(0);
        robotHardware.cleanser.setPower(0);

    }
}
