package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.lang.annotation.ElementType;
import java.security.spec.EllipticCurve;

public class RobotHardware {

    // declare imu
    BNO055IMU imu;

    //declaring variables to wait in code
    private ElapsedTime waitTime = new ElapsedTime();
    Constants constants = new Constants();

    //declaring motors
    //shooter motor
    public DcMotorEx shooter;
    //drive motors
    public DcMotor fr, fl, br, bl;
    //"watermill" motor
    public DcMotor cleanser;
    //intake
    public DcMotor intake;

    //servos
    public Servo wobbleClamp;
    public Servo wobbleArm;

    LinearOpMode op;
    Telemetry telemetry;

    HardwareMap hwMap = null;

    //constructor
    public RobotHardware(LinearOpMode op, Telemetry telemetry) {
        this.op = op;
        this.telemetry = telemetry;
    }

    //gets imu angle
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }
    //initialization
    public void init (HardwareMap hwMap, boolean initServos) {
        this.hwMap = hwMap;
        //init motors/servos
        //shooter motor
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        //drive motors
        fr = hwMap.get(DcMotor.class, "fr");
        fl = hwMap.get(DcMotor.class, "fl");
        br = hwMap.get(DcMotor.class, "br");
        bl = hwMap.get(DcMotor.class, "bl");
        //"watermill" motor
        cleanser = hwMap.get(DcMotor.class, "cleanser");
        //intake motor
        intake = hwMap.get(DcMotor.class, "intake");
        //wobble arm
        wobbleArm = hwMap.get(Servo.class, "wobbleArm");
        wobbleClamp = hwMap.get(Servo.class, "wobbleClamp");

        //reset encoders
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set motor mode
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cleanser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //zero power behavior
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cleanser.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting direction of motors (none of them were black to red and red to black don't worry)
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        cleanser.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        //set power to 0 on init
        shooter.setPower(0);
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        cleanser.setPower(0);
        intake.setPower(0);

        //init servos
        if (initServos) {
            initServos();
        }
        //else don't

        //gyro/imu
        imu = hwMap.get(BNO055IMU.class, "imu");
        resetGyro();

        telemetry.addLine("Ready to Start");
        telemetry.update();

    }

    //initialize Servos
    public void initServos() {
        wobbleArm.setPosition(constants.armUp);
        wobbleClamp.setPosition(constants.clampUp);
    }

    //reset imu
    public void resetGyro() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);

    }

    //reset drive encoders
    public void resetEncoders() {

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //wait for given milliseconds
    public void sleep(long ms) {
        ElapsedTime waitTime = new ElapsedTime();
        waitTime.reset();
        while (waitTime.milliseconds() < ms && op.opModeIsActive()) {
            op.idle();
        }
    }

    //basic movement for given power and milliseconds
    public void drive(double p, int ms) {
        fr.setPower(p);
        fl.setPower(p);
        br.setPower(p);
        bl.setPower(p);

        ElapsedTime elapsedTime = new ElapsedTime();
        double endTime = elapsedTime.milliseconds() + ms;
        float angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (elapsedTime.milliseconds() < endTime && op.opModeIsActive()) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            fr.setPower(p - (p * (angle / 30)));
            fl.setPower(p + (p * (angle / 30)));
            br.setPower(p - (p * (angle / 30)));
            bl.setPower(p + (p * (angle / 30)));
        }

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void strafe(double p, int ms) {
        //to go right:
        //  fl and br forward
        //  fr and bl backward
        fr.setPower(-p);
        fl.setPower(p);
        br.setPower(p);
        bl.setPower(-p);
        sleep(ms);
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void turn(double p, int ms) {
        fr.setPower(-p);
        fl.setPower(p);
        br.setPower(-p);
        bl.setPower(p);
        sleep(ms);
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void brake() {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    //turn to given angle based on imu
    public void turnTo(int angle) {
        angle = -angle;
        float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (Math.abs(currentAngle - angle) > 1 && op.opModeIsActive()) {
            if (angle < currentAngle) {
                turn(0.25, 50);
            }
            else {
                turn(-0.25, 50);
            }

            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            telemetry.addData("imu", currentAngle);
            telemetry.update();
        }
        sleep(250);
    }
    //move based on encoders and imu correction
    public void moveTo(int x, int y) {
        resetEncoders();

//        x*= 1.3;
//        y*= 1.3;
        double angle = Math.atan((double)x/y) / Math.PI * 180;
//        if (y < 0)
//            angle = x >= 0 ? angle + 180 : angle - 180;
        double hypotenuse = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double xPower = x/hypotenuse;
        double yPower = y/hypotenuse;
        double cos = Math.cos(angle * Math.PI / 90);
        double neededTicks = hypotenuse * constants.ticksPerTok;
        double currentTicks = angle >= 0 ? fl.getCurrentPosition() : fr.getCurrentPosition();

        while (neededTicks > currentTicks && op.opModeIsActive()) {
            currentTicks = angle >= 0 ? -fl.getCurrentPosition() : fr.getCurrentPosition();
            double power = -4 / Math.pow(neededTicks, 2) * Math.pow((currentTicks - neededTicks/2), 2) + 1;
            power = power < 0.25 && currentTicks < neededTicks / 2 ? 0.25 : power;
            (angle >= 0 ? fl : fr).setPower(power);
            (angle >= 0 ? br : bl).setPower(power);

            (angle >= 0 ? fr : fl).setPower(power * cos);
            (angle >= 0 ? bl : br).setPower(power * cos);

            telemetry.addData("angle", angle);
            telemetry.addData("needed ticks", neededTicks);
            telemetry.addData("current ticks", currentTicks);
            telemetry.addData("power", power);
            telemetry.update();
        }
        sleep(1000);
    }
    public void driveTo(int inches) {
        resetEncoders();
        double neededTicks = inches * constants.ticksPerTok;
        double currentTicks = fr.getCurrentPosition();
        double initialAngle = getAngle();
        while (Math.abs(currentTicks - neededTicks) > 50 && op.opModeIsActive()) {
            currentTicks = fr.getCurrentPosition();
            //standard deviation-like power curve:
            //  e^(-(x-mu)^2/a)
            //  starts and ends at 2 standard deviations away from the mean
            //  centered at mu = neededTicks/2
            //  stretched using a = -mu^2/ln(min)
            //  the minimum amount of power it will experience is min = 0.25
//           ___
//         _/   \_
//      __/       \__
//   __/             \__

//            double mu = neededTicks / 2;
//            double a = -Math.pow(mu, 2) / Math.log(0.25);
//            double power = (inches < 24 ? 0.25 : 0.6) * Math.pow(Math.E, (-Math.pow((currentTicks - mu), 2) / (a)));
            //0.25 power
//            double power = 0.25;
//   ------------
            //least of two lines
//   \  /
//    \/
//    /\
//   /  \
            double power1 = 0.25 + ((1.0 / 567) * currentTicks);
            double power2 = -((1.0 / 567) * currentTicks) + (0.25 + -((1.0 / 567) * currentTicks));
            double power = power1 < power2 ? power1 : power2;
            double currentAngle = getAngle();

            power = power < 0.2 && currentTicks > neededTicks ? -0.2 : power < 0.2 ? 0.2 : power;

            double correction = (currentAngle - initialAngle) / 30;

            fr.setPower(power - correction);
            fl.setPower(power + correction);
            br.setPower(power - correction);
            bl.setPower(power + correction);

            telemetry.addData("needed", neededTicks);
            telemetry.addData("current", currentTicks);
            telemetry.addData("power", power);
            telemetry.update();
        }
        brake();
    }
    public void strafeTo(int inches) {
        resetEncoders();
        double neededTicks = inches * constants.ticksPerTok;
        double currentTicks = fr.getCurrentPosition();
        double initialAngle = getAngle();
        while (Math.abs(currentTicks - neededTicks) > 50 && op.opModeIsActive()) {
            currentTicks = fr.getCurrentPosition();

            double power1 = 0.25 + ((1.0 / 567) * currentTicks);
            double power2 = -((1.0 / 567) * currentTicks) + (0.25 + -((1.0 / 567) * currentTicks));
            double power = power1 < power2 ? power1 : power2;
            double currentAngle = getAngle();

            power = power < 0.2 && currentTicks > neededTicks ? -0.2 : power < 0.2 ? 0.2 : power;

            double correction = (currentAngle - initialAngle) / 30;

            fr.setPower(power - correction);
            fl.setPower(-power + correction);
            br.setPower(-power - correction);
            bl.setPower(power + correction);

            telemetry.addData("needed", neededTicks);
            telemetry.addData("current", currentTicks);
            telemetry.addData("power", power);
            telemetry.update();
        }
        brake();
    }
}









