package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Movement {

    LinearOpMode op;
    RobotHardware robotHardware;
    Telemetry telemetry;
    Constants constants = new Constants();

    public double strafe = 0;
    public double drive = 0;
    public double turn = 0;
    public double ix, iy, dx, dy, prevX, prevY = 0;

    public Movement(LinearOpMode op, RobotHardware robotHardware, Telemetry telemetry) {
        this.op = op;
        this.robotHardware = robotHardware;
        this.telemetry = telemetry;
    }

    public void setPowers() {
        setPowers(constants.shooterPower);
    }
    public void setPowers(double shooterPower) {
        //+ = forward, right, ccw
        robotHardware.shooter.setVelocity(shooterPower);
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

    public void turnTo(double angle, BackboardPipeline pipeline) {
        turn = (pipeline.getAngle() - angle) / 60;
    }
    public void turnTo(double angle) {
        double currentAngle = robotHardware.getAngle();
        //        double power = (angle == 180) ? (currentAngle >= 0 ? currentAngle - 180 : 180 + currentAngle) : currentAngle - angle;

        turn = (angle - currentAngle) / 40;
    }
    public void turnToX(int x, BackboardPipeline pipeline) {
        if (pipeline.detected()) {
            turn = (x - pipeline.getX()) / 300.0;
        }
    }

    public void goToPoint(int x, int y, int currentX, int currentY, BackboardPipeline pipeline, double power1, double power2) {
        //PID controller go to point

        //constants
        double kp = 1 / 80.0;
        double ki = 1 / 5000.0;
        double kd = 1 / 400.0;

        //proportional(delta x/y)
        double deltaX = currentX - x;
        double deltaY = currentY - y;
        //proportional power value
        double px = deltaX * kp;
        double py = deltaY * kp;

        //integral controller if within 20 units from intended spot
        if (deltaX > 15)
            ix = 0;
        else
            ix+= deltaX * ki;
        if (deltaY > 15)
            iy = 0;
        else
            iy+= deltaY * ki;

        //derivative controller
        dx = (x - prevX) * kd;
        dy = (y - prevY) * kd;
        prevX = currentX;
        prevY = currentY;

        //set powers
//        strafe = px + ix - dx;
//        drive = py + iy - dy;
        strafe = px + dx;
        drive = py + dy;
        strafe*= power2;
        drive*= power1;
    }
    public void goToPoint(int x, int y, BackboardPipeline pipeline, boolean straight, double power1, double power2) {
        if (straight) {
            turnTo(0);
        }
        if (pipeline.detected()) {
            goToPoint(x, y, pipeline.getX(), pipeline.getY(), pipeline, power1, power2);
        }
//        else
//            goToPoint(x, y, prevX, prevY, pipeline, straight);
    }
    public void goToPoint(int x, int y, BackboardPipeline pipeline) {
        goToPoint(x, y, pipeline, 1);
    }
    public void goToPoint(int x, int y, BackboardPipeline pipeline, double power) {
        goToPoint(x, y, pipeline, true, power, power);
    }
    public void goToPoint(int x, int y, BackboardPipeline pipeline, double power1, double power2) {
        goToPoint(x, y, pipeline, true, power1, power2);
    }
    public void shoot(int velocity) {
        robotHardware.shooter.setVelocity(velocity);

        if (Math.abs(robotHardware.shooter.getVelocity() - velocity) < 200) {
            robotHardware.blocker.setPosition(constants.blockerUp);
            robotHardware.intake.setPower(0.5);
            robotHardware.cleanser.setPower(0.5);
        }
        else {
            robotHardware.intake.setPower(0);
            robotHardware.cleanser.setPower(0);
        }
    }
    public void shoot() {
        shoot(constants.shooterPower);
    }
//    public void shooterOff() {
//        robotHardware.shooter.setVelocity(0);
//        robotHardware.intakeOff();
//    }

    public boolean closeTo(int x, int y, int closeness, BackboardPipeline pipeline) {
        return Math.sqrt(Math.abs(Math.pow(x- pipeline.getX(), 2) + Math.pow(y - pipeline.getY(), 2))) <= closeness;

    }
    public boolean closeTo(int x, int y, BackboardPipeline pipeline) {
        return closeTo(x, y, 25, pipeline);
    }
    public double speed() {
        return Math.abs(drive) + Math.abs(turn) + Math.abs(strafe);
    }

    public void block() {
        robotHardware.blocker.setPosition(constants.blockerDown);
    }



}
