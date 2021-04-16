package org.firstinspires.ftc.teamcode;

import android.text.method.BaseKeyListener;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Movement {

    LinearOpMode op;
    RobotHardware robotHardware;
    Telemetry telemetry;
    Constants constants = new Constants();

    public double strafe = 0;
    public double drive = 0;
    public double turn = 0;

    public Movement(LinearOpMode op, RobotHardware robotHardware, Telemetry telemetry) {
        this.op = op;
        this.robotHardware = robotHardware;
        this.telemetry = telemetry;
    }

    public void setPowers() {
        //+ = forward, right, ccw
        robotHardware.fr.setPower(drive - strafe + turn);
        robotHardware.fl.setPower(drive + strafe - turn);
        robotHardware.br.setPower(drive + strafe + turn);
        robotHardware.bl.setPower(drive - strafe - turn);
    }
    public void resetPower() {
        strafe = 0;
        drive = 0;
        turn = 0;
    }
    public void turnTo(double angle) {
        turn = (angle - robotHardware.getAngle()) / 40;
    }

    public void goToPoint(int x, int y, BackboardPipeline pipeline, boolean straight) {
        if (pipeline.detected()) {
            strafe = (pipeline.getX() - x) / 100.0;
            drive = (pipeline.getY() - y) / 100.0;
            turnTo(0);
        }
    }
    public void goToPoint(int x, int y, BackboardPipeline pipeline) {
        goToPoint(x, y, pipeline, true);
    }

    public void shoot(int power) {
        robotHardware.shooter.setVelocity(power);
        robotHardware.blocker.setPosition(constants.blockerUp);
        if (Math.abs(robotHardware.shooter.getVelocity() - power) < 200)
            robotHardware.intakeOn();
        else
            robotHardware.intakeOff();
    }
    public void shoot() {
        shoot(constants.shooterPower);
    }
    public void shooterOff() {
        robotHardware.shooter.setVelocity(0);
        robotHardware.intakeOff();
    }

    public boolean closeTo(int x, int y, int closeness, BackboardPipeline pipeline) {
        return Math.sqrt(Math.abs(Math.pow(x - pipeline.getX(), 2) + Math.pow(y - pipeline.getY(), 2))) < closeness;

    }
    public boolean closeTo(int x, int y, BackboardPipeline pipeline) {
        return closeTo(x, y, 25, pipeline);
    }
    public double speed() {
        return Math.abs(drive) + Math.abs(turn) + Math.abs(strafe);
    }



}
