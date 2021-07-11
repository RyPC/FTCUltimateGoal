package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

public class RobotHardware {

    // declare imu
    BNO055IMU imu;

    //random constants
    Constants constants = new Constants();

    //camera view in phone
//    int cameraMonitorViewId;
    //declaring camera
    public WebcamName backboardWebcam;
    public WebcamName ringWebcam;
    public OpenCvCamera backboardCamera;
    public OpenCvCamera ringCamera;

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
    public Servo blocker;
    public Servo leftEgg;
    public Servo rightEgg;

    public double angleAdjustment = 0;
    public int pastVelocity = 0;

    LinearOpMode op;
    Telemetry telemetry;

    public HardwareMap hwMap = null;

    //constructor
    public RobotHardware(LinearOpMode op, Telemetry telemetry) {
        this.op = op;
        this.telemetry = telemetry;
    }

    //gets imu angle
    public double getRealAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }
    public double getAngle() {
        return getRealAngle() + angleAdjustment;
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
        //shooter blocker
        blocker = hwMap.get(Servo.class, "blocker");
        //eggs
        leftEgg = hwMap.get(Servo.class, "leftEgg");
        rightEgg = hwMap.get(Servo.class, "rightEgg");
        //cameras
        backboardWebcam = hwMap.get(WebcamName.class, "Looky");
        ringWebcam = hwMap.get(WebcamName.class, "Looky2");

        //reset encoders
        resetEncoders();

        //set motor mode
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        cleanser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //zero power behavior
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cleanser.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting direction of motors (none of them were black to red and red to black don't worry)
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
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

        //live cameras (used in other files)
//        backboardCamera = OpenCvCameraFactory.getInstance().createWebcam(backboardWebcam);
//        ringCamera = OpenCvCameraFactory.getInstance().createWebcam(ringWebcam);

        telemetry.addLine("Ready to Start");
        telemetry.update();

    }

    //initialize Servos
    public void initServos() {
        wobbleArm.setPosition(constants.armUp);
        wobbleClamp.setPosition(constants.clampClosed);
        blocker.setPosition(constants.blockerDown);
        rightEgg.setPosition(constants.rightEggUp);
        leftEgg.setPosition(constants.leftEggUp);
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
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        //clockwise - positive
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
    public void turnTo(int angle, double turnSpeed) {
        double currentAngle = getAngle();
        while (Math.abs(currentAngle - angle) > 1 && op.opModeIsActive()) {


            //least of two power lines

            double power;

            power = (angle == 180) ? (currentAngle >= 0 ? currentAngle - 180 : 180 + currentAngle) : currentAngle - angle;
            power/= (30 / turnSpeed);
            if (power < 0.15 && power > 0)
                power = 0.15;
            else if (power > -0.25 && power < 0)
                power = -0.15;

            fr.setPower(-power);
            fl.setPower(power);
            br.setPower(-power);
            bl.setPower(power);

            currentAngle = getAngle();

            telemetry.addData("imu", currentAngle);
            telemetry.addData("power", power);
            telemetry.update();
        }

        brake();
        sleep(250);
    }
    public void turnTo(int angle) {
        double currentAngle = getAngle();
        double power = Math.abs(currentAngle - angle) > 135 ? 0.35 : Math.abs(currentAngle - angle) > 10 ? 1 : 2;
        turnTo(angle, power);
    }

    //turn using a PID Controller
    public void turnPID(int angle) {
        //PID Controller: Proportional Integral Derivative Controller
        //P: Power | R: integral of the error from t=0 to t=t | E: Error | Ep = Previous Error | t = time from last check
        //P = (kp * E) + (ki * R) + (kd * (E - Ep) / t)

        //k constants for proportional, integral, and derivative parts
        double kp = 0.01;
        int ki = 0;
        int kd = 0;
        //time from start to find the change in time
        ElapsedTime t = new ElapsedTime();
        double currentAngle = getAngle();
        //declare other variables
        double P = 0;
        int R = 0;
        double E;
        double Ep = currentAngle - angle;

        t.reset();
        while ((Math.abs(angle - currentAngle) > 1 || Math.abs(P) < 0.2) && op.opModeIsActive()) {
            telemetry.addData("power", P);
            telemetry.update();
            E = currentAngle - angle;
            R+= E * t.milliseconds() / 1000.0;

            //set motor powers to turn
            P = (kp * E) + (ki * R) + (kd * (E - Ep) / (t.milliseconds() / 1000.0));
            fr.setPower(-P);
            br.setPower(-P);
            fl.setPower(P);
            bl.setPower(P);

            Ep = (double)E;
            currentAngle = getAngle();
            t.reset();
        }
        brake();
    }

    //move based on encoders and imu correction
    public void moveTo(int x, int y) {
        resetEncoders();

        //angle of path taken
        double angle = Math.atan((double)x / y) / Math.PI * 180;
        //total distance need to be traveled determined by \sqrt(x^2 + y^2)
        double hypotenuse = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        //initial angle of the robot
        double initialAngle = getAngle();

//        double xPower = x/hypotenuse;
//        double yPower = y/hypotenuse;

        //cos needed to find the power output of the different motors
        double cos = Math.cos(angle * 2 * Math.PI / 360 * 2);

        //neededTicks: ticks needed to reach the end of the path
        //currentTicks: current number of ticks traveled from the start of the moveTo() function
        double neededTicks = hypotenuse * constants.ticksPerTok;
        double currentTicks = 0;

        while (neededTicks > currentTicks && op.opModeIsActive()) {

            currentTicks = angle >= 0 ? fl.getCurrentPosition() : -fr.getCurrentPosition();

            //least of two power lines
            double power1 = 0.25 + ((1.0 / 1890) * currentTicks);
            double power2 = -((1.0 / 1890) * currentTicks) + (0.25 + neededTicks / 1890);
            double power = Math.min(power1, power2);

            //correction based on imu
            double currentAngle = getAngle();
            double correction = (currentAngle - initialAngle) / 30;

            //sets power of 4 motors
            power = power < 0.25 && currentTicks < neededTicks / 2 ? 0.25 : power < 0 ? -0.25 : power;
            double highPower = power * (angle < 0 ? 1 : cos) - correction;
            double lowPower = power * (angle < 0 ? cos : 1) + correction;
            fr.setPower(highPower);
            fl.setPower(lowPower);
            br.setPower(highPower);
            bl.setPower(lowPower);

            //telemetry
            telemetry.addData("angle", angle);
            telemetry.addLine(currentTicks + " / " + neededTicks + " ticks");
            telemetry.addData("high", highPower);
            telemetry.addData("low", lowPower);

            telemetry.update();
        }
        brake();
    }

    public void driveTo(int inches, boolean shoot, int angle) {
        resetEncoders();
        double neededTicks = inches * constants.ticksPerTok;
        double currentTicks = -fr.getCurrentPosition();
        while (Math.abs(currentTicks - neededTicks) > 50 && op.opModeIsActive()) {
            currentTicks = -fr.getCurrentPosition();
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
            double power1 = 0.25 + (currentTicks / 1890.0);
            double power2 = -(currentTicks / 1890.0) + (0.25 + neededTicks / 1890.0);
            double power = Math.min(power1, power2);
            double currentAngle = getAngle();

            power = power < 0.25 && currentTicks > neededTicks ? -0.25 : Math.max(power, 0.25);

            //sets correction for motor power based off of imu
            double correction = (currentAngle - (double) angle) / 30;

            //sets motor power
            fr.setPower(power - correction);
            fl.setPower(power + correction);
            br.setPower(power - correction);
            bl.setPower(power + correction);

            //shoots if shooter is at speed
            if (shoot) {
                if (Math.abs(shooter.getVelocity() - constants.shooterPower) < 75) {
                    cleanser.setPower(0.25);
                    intake.setPower(0.5);
                } else {
                    cleanser.setPower(0);
                    intake.setPower(0);
                }
            }


            telemetry.addData("needed", neededTicks);
            telemetry.addData("current", currentTicks);
            telemetry.addData("power", power);
            telemetry.addData("angle", (double) angle);
            telemetry.update();
        }
        brake();
    }
    public void driveTo(int inches) {
        driveTo(inches, false, 0);
    }
    public void driveTo(int inches, int angle) {
        driveTo(inches, false, angle);
    }
    public void strafeTo(int inches, int angle) {
        //right positive
        ElapsedTime time = new ElapsedTime();
        time.reset();

        inches*= 1.33;
        resetEncoders();
        double neededTicks = inches * constants.ticksPerTok;
        double currentTicks = 0;
        while (Math.abs(currentTicks - neededTicks) > 50 && op.opModeIsActive() && time.milliseconds() < 5000) {
            currentTicks = fr.getCurrentPosition();

//            double power1 = 0.25 + ((1.0 / 1890) * currentTicks);
//            double power2 = -((1.0 / 1890) * currentTicks) + (0.25 + neededTicks / 1890);
//            double power = inches > 0 ? Math.min(power1, power2) : Math.max(power1, power2);
            double power = inches < 0 ? -0.5 : 0.5;
            double currentAngle = getAngle();

//            power = power > 0 ? Math.max(0.5, power) : Math.min(-0.5, power);

            double correction = (currentAngle - angle) / 20;

            fr.setPower(-power - correction);
            fl.setPower(power + correction);
            br.setPower(power - correction);
            bl.setPower(-power + correction);

//            telemetry.addData("needed", neededTicks);
//            telemetry.addData("current", currentTicks);
//            telemetry.addData("power", power);
//            telemetry.update();
        }
        brake();
    }
    public void strafeTo(int inches) {
        strafeTo(inches, 0);
    }

    //strafe at constant power
    public void strafePower(int inches, double power) {
        resetEncoders();
        double neededTicks = inches * constants.ticksPerTok;
        double currentTicks = -fr.getCurrentPosition();
        while (Math.abs(currentTicks - neededTicks) > 50 && op.opModeIsActive()) {
            currentTicks = -fr.getCurrentPosition();

            double currentAngle = getAngle();

            double correction = (Math.abs(currentAngle) > 160) ? ((180 - Math.abs(currentAngle)) * Math.abs(currentAngle) / currentAngle / 30) : 0;

            fr.setPower(-power + correction);
            fl.setPower(power - correction);
            br.setPower(power + correction);
            bl.setPower(-power - correction);
        }
        brake();
    }

    //drive at constant power
    public void drivePower(int inches, double power, boolean shoot, int angle, int shooterPower) {
        resetEncoders();
        double neededTicks = inches * constants.ticksPerTok;
        double currentTicks = fr.getCurrentPosition();
        while (Math.abs(currentTicks - neededTicks) > 50 && op.opModeIsActive()) {
            currentTicks = fr.getCurrentPosition();

            double currentAngle = getAngle();

            //sets correction for motor power based off of imu
            double correction = (currentAngle - (double) angle) / 25;

            //sets motor power
            fr.setPower(power - correction);
            fl.setPower(power + correction);
            br.setPower(power - correction);
            bl.setPower(power + correction);
            shooter.setVelocity(shooterPower);

            //shoots if shooter is at speed
            if (shoot) {
                if (Math.abs(shooter.getVelocity() - shooterPower) < 75) {
                    cleanser.setPower(0.33);
                    intake.setPower(0.5);
                } else {
                    cleanser.setPower(0);
                    intake.setPower(0);
                }
            }

            telemetry.addData("needed", neededTicks);
            telemetry.addData("current", currentTicks);
            telemetry.addData("angle", (double) angle);
            telemetry.update();
        }
        brake();
    }
    public void drivePower(int inches, double power, int angle) {
        drivePower(inches, power, false, angle, constants.shooterPower);
    }
    public void drivePower(int inches, double power) {
        blocker.setPosition(constants.blockerDown);
        drivePower(inches, power, false, 0, constants.shooterPower);
    }

    //shoot for given time
    public void shoot(double ms, double shooterPower) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        shooter.setVelocity(shooterPower);
        double intakePower = intake.getPower();
        while (time.milliseconds() < ms && op.opModeIsActive()) {
            //  won't fire rings unless shooter is at correct velocity
            if (Math.abs(shooter.getVelocity() - shooterPower) < 50) {
                cleanser.setPower(0.33);
                intake.setPower(0.5);
            }
            else {
                cleanser.setPower(0);
                intake.setPower(intakePower);
            }
        }
        intake.setPower(intakePower);
        cleanser.setPower(0);
    }
    public void shoot(double ms) {
        shoot(ms, constants.shooterPower);
    }

    //shoots 3 rings quickly
    public void shootRings(double power) {
        blocker.setPosition(constants.blockerDown);
        shooter.setVelocity(power);
        while (Math.abs(shooter.getVelocity() - power) <= 10 && op.opModeIsActive()) {
            op.idle();
        }
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        blocker.setPosition(constants.blockerUp);
        while (elapsedTime.milliseconds() <= 1000 && op.opModeIsActive()) {
            intake.setPower(0.75);
            cleanser.setPower(0.75);
        }
        intake.setPower(0);
        cleanser.setPower(0);
        blocker.setPosition(constants.blockerUp);
    }
    public void shootRings() {
        shootRings(constants.shooterPower);
    }

    public void placeWobble () {
        wobbleArm.setPosition(constants.armDown);
        sleep(350);
        wobbleClamp.setPosition(constants.clampOpen);
        sleep(250);
        wobbleArm.setPosition(constants.armUp);
        wobbleClamp.setPosition(constants.clampClosed);
    }

    public void intake(boolean on) {
        intake.setPower(on ? 1 : 0);
        cleanser.setPower(on ? 1 : 0);
    }
    public void intakeOn() {
        intake(true);
    }
    public void intakeOff() {
        intake(false);
    }

    public void eggsDown() {
        rightEgg.setPosition(constants.rightEggDown);
        leftEgg.setPosition(constants.leftEggDown);
    }
    public void eggsUp() {
        rightEgg.setPosition(constants.rightEggUp);
        leftEgg.setPosition(constants.leftEggUp);
    }

    //PID controller for shooter
    public void setShooter(int velocity) {
        int currentVelocity = (int) shooter.getVelocity();
        int pastVelocity = this.pastVelocity;

        double proportional = (velocity - currentVelocity) / 1000.0;
        double derivative = (currentVelocity - pastVelocity) / 1000.0;
        double integral = 0;
        double power = proportional;
        shooter.setPower(power);


        this.pastVelocity = currentVelocity;
    }

}








