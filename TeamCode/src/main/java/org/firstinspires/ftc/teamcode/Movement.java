package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Movement {

    LinearOpMode op;
    RobotHardware robotHardware;
    Telemetry telemetry;
    Constants constants = new Constants();

    public double strafe = 0;
    public int prevX, prevY;
    public double drive = 0;
    public double turn = 0;
    public double is, id;

    public Movement(LinearOpMode op, RobotHardware robotHardware, Telemetry telemetry) {
        this.op = op;
        this.robotHardware = robotHardware;
        this.telemetry = telemetry;
    }

    public void setPowers() {
        //+ = forward, right, ccw
        robotHardware.shooter.setVelocity(constants.shooterPower);
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
        double power = (angle - currentAngle) / 40;
//        double power = (angle == 180) ? (currentAngle >= 0 ? currentAngle - 180 : 180 + currentAngle) : currentAngle - angle;

        turn = power;
    }
    public void turnToX(int x, BackboardPipeline pipeline) {
        if (pipeline.detected()) {
            turn = (x - pipeline.getX()) / 300.0;
        }
    }

    public void goToPoint(int x, int y, int currentX, int currentY, BackboardPipeline pipeline, double power1, double power2) {
        //PI controller go to point
        double diffs = currentX - x;
        double diffd = currentY - y;
        double ps = diffs / 80.0;
        double pd = diffd / 80.0;
        is = ((closeTo(x, currentY, 15, pipeline) && !closeTo(x, currentY, 0, pipeline)) ? is + diffd : 0) / 90;
        id = ((closeTo(currentX, y, 15, pipeline) && !closeTo(currentX, y, 0, pipeline)) ? id + diffs : 0) / 90;
        strafe = ps;
        drive = pd;
        strafe*= power2;
        drive*= power1;
    }
    public void goToPoint(int x, int y, BackboardPipeline pipeline, boolean straight, double power1, double power2) {
        if (straight) {
            turnTo(0);
        }
        if (pipeline.detected()) {
            prevX = pipeline.getX();
            prevY = pipeline.getY();
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
        return Math.sqrt(Math.abs(Math.pow(x - pipeline.getX(), 2) + Math.pow(y - pipeline.getY(), 2))) <= closeness;

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
