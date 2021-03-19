package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RobotHardware {

    // declare imu
    BNO055IMU imu;

    //declaring variables to wait in code
    private ElapsedTime waitTime = new ElapsedTime();

    //declaring motors
    //shooter motor
    public DcMotorEx shooter;
    //drive motors
    public DcMotor fr, fl, br, bl;
    //"watermill" motor
    public DcMotor cleanser;
    //intake
    public DcMotor intake;

    LinearOpMode op;
    Telemetry telemetry;

    HardwareMap hwMap = null;

    public RobotHardware(LinearOpMode op, Telemetry telemetry) {
        this.op = op;
        this.telemetry = telemetry;
    }

    //initialization
    public void init (HardwareMap hwmap, boolean initServos) {
        this.hwMap = hwmap;
        //init motors
        //shooter motor
        shooter = hwmap.get(DcMotorEx.class, "shooter");
        //drive motors
        fr = hwmap.get(DcMotor.class, "fr");
        fl = hwmap.get(DcMotor.class, "fl");
        br = hwmap.get(DcMotor.class, "br");
        bl = hwmap.get(DcMotor.class, "bl");
        //"watermill" motor
        cleanser = hwmap.get(DcMotor.class, "cleanser");
        //intake
        intake = hwmap.get(DcMotor.class, "intake");

        //set motor mode
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void resetGyro() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);

    }

    public void initServos() {
        //servo.setPosition();
    }

    //wait for given milliseconds
    public void sleep(long ms) {
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime waitTime = new ElapsedTime();
        waitTime.reset();
        while (waitTime.milliseconds() < ms && op.opModeIsActive()) {
            op.idle();
        }
    }

    //movement for given power and milliseconds
    public void drive(double p, int ms) {
        fr.setPower(p);
        fl.setPower(p);
        br.setPower(p);
        bl.setPower(p);
        sleep(ms);
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void strafe(double p, int ms) {
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
    public void turnTo(int angle) {
        float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (Math.abs(currentAngle - angle) > 2 && op.opModeIsActive()) {
            turn(0.25, 200);
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("imu", currentAngle);
            telemetry.update();
        }
        sleep(250);
    }

}











