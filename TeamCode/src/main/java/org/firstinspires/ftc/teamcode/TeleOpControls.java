package org.firstinspires.ftc.teamcode;
//LSErljaew;ljfs;jl [-[enise jaand the jafucking bitch
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TeleOpControls {
    BackboardPipeline pipeline;
    LinearOpMode op;
    RobotHardware robotHardware;
    Telemetry telemetry;
    Constants constants = new Constants();
    int shooterSpeed = constants.shooterPower + 300;
    boolean dPDPressed = false;
    boolean dPDDown = false;
    boolean rbPressed = true;
    boolean rbDown = false;
    boolean dPRDown = false;
    boolean dPLDown = false;
    boolean lbPressed = true;
    boolean lbDown = false;
    boolean bPressed = false;
    boolean bDown = false;
    boolean eggDown = false;
    boolean eggPressed = false;
    boolean gateOpen = false;
    boolean aDown = false;
    boolean xPressed = false;
    boolean xDown = false;
    ElapsedTime shootTime = new ElapsedTime();


    public TeleOpControls(LinearOpMode op, RobotHardware robotHardware, Telemetry telemetry) {
        this.op = op;
        this.robotHardware = robotHardware;
        this.telemetry = telemetry;
    }

    public void notDriving () {


        //intake/cleanser
        //  right stick
        // outtake for a bit before shooting rings using 'a'
        if (op.gamepad1.a && !aDown) {
            aDown = true;
            shootTime.reset();
        }
        else if (!op.gamepad1.a && aDown) {
            aDown = false;
        }
        //outtake
//        robotHardware.intake.setPower(op.gamepad1.a ? (shootTime.milliseconds() < 150 ? -1 : 1) : op.gamepad1.right_stick_y);
//        robotHardware.cleanser.setPower(op.gamepad1.a ? (shootTime.milliseconds() < 150 ? -1 : 1) : op.gamepad1.right_stick_y * 0.75);
        //no outtake
        robotHardware.intake.setPower(op.gamepad1.a ? 1 : op.gamepad1.right_stick_y);
        robotHardware.cleanser.setPower(op.gamepad1.a ? 1 : op.gamepad1.right_stick_y * 0.75);



        //shooter
        //  A toggle on/off
        //  on/off shooter
        if (op.gamepad1.dpad_down && !dPDDown) {
            dPDDown = true;
            dPDPressed = !dPDPressed;
        }
        else if (!op.gamepad1.dpad_down && dPDDown) {
            dPDDown = false;
        }
        robotHardware.shooter.setVelocity(dPDPressed ? 0 : shooterSpeed);

        //wobble goal
        //  LB toggle down/up
        //  RB toggle clamp/release
        if (op.gamepad1.right_bumper && !rbDown) {
            rbDown = true;
            rbPressed = !rbPressed;
        }
        else if (!op.gamepad1.right_bumper && rbDown) {
            rbDown = false;
        }
        robotHardware.wobbleClamp.setPosition(rbPressed ? constants.clampClosed : constants.clampOpen);

        if (op.gamepad1.left_bumper && !lbDown) {
            lbDown = true;
            lbPressed = !lbPressed;
            rbPressed = lbPressed;
        }
        else if (!op.gamepad1.left_bumper && lbDown) {
            lbDown = false;
        }
        robotHardware.wobbleArm.setPosition(lbPressed ? constants.armUp : constants.armDown);

        //power shots mode
        //  lowers power
        if (op.gamepad1.b && !bDown) {
            bDown = true;
            bPressed = !bPressed;
        }
        else if (!op.gamepad1.b && bDown) {
            bDown = false;
        }

        shooterSpeed = bPressed ? constants.shooterPower - 180: constants.shooterPower + 40;

        //changing angle with dpad right/left
        if (op.gamepad1.dpad_right && !dPRDown) {
            dPRDown = true;
            robotHardware.angleAdjustment+= 2;
        }
        else if (!op.gamepad1.dpad_right)
            dPRDown = false;
        if (op.gamepad1.dpad_left && !dPLDown) {
            dPLDown = true;
            robotHardware.angleAdjustment-= 2;
        }
        else if (!op.gamepad1.dpad_left)
            dPLDown = false;
        if (op.gamepad1.dpad_up)
            robotHardware.angleAdjustment = -robotHardware.getRealAngle();

    }
    public void checkBlocker() {
        if (!gateOpen && Math.abs(robotHardware.shooter.getVelocity() - shooterSpeed) <= 10 && op.gamepad1.a)
            gateOpen = true;
        else if (gateOpen && !op.gamepad1.a)
            gateOpen = false;
        robotHardware.blocker.setPosition(gateOpen ? constants.blockerUp : constants.blockerDown);
    }
    public void noCheckBlocker() {
        robotHardware.blocker.setPosition(op.gamepad1.a ? constants.blockerUp : constants.blockerDown);
    }

    public void normalDrive() {
        //movement
        //  left stick: strafing/movement
        //  triggers: rotation
        //  x toggle slow mode: 30% speed

        if (op.gamepad1.x && !xDown) {
            xDown = true;
            xPressed = !xPressed;
        }
        else if (!op.gamepad1.x && xDown) {
            xDown = false;
        }

        double forward = -op.gamepad1.left_stick_y;
        double strafe = op.gamepad1.left_stick_x;
        double rotate = op.gamepad1.right_trigger - op.gamepad1.left_trigger;
        double multiplier = (xPressed ? 0.1 : 1);

        robotHardware.fr.setPower(multiplier * (forward - rotate - strafe));
        robotHardware.fl.setPower(multiplier * (forward + rotate + strafe));
        robotHardware.br.setPower(multiplier * (forward - rotate + strafe));
        robotHardware.bl.setPower(multiplier * (forward + rotate - strafe));
    }

    public void weirdDrive() {
        //movement
        //  left stick: strafing/movement
        //  right stick x: rotation
        double forward = -op.gamepad1.left_stick_y;
        double strafe = op.gamepad1.left_stick_x;
        double rotate = op.gamepad1.right_stick_x;
        double multiplier = 1;

        robotHardware.fr.setPower(multiplier * (forward - rotate - strafe));
        robotHardware.fl.setPower(multiplier * (forward + rotate + strafe));
        robotHardware.br.setPower(multiplier * (forward - rotate + strafe));
        robotHardware.bl.setPower(multiplier * (forward + rotate - strafe));
    }

    public void useCamera() {
        pipeline = new BackboardPipeline(Color.RED);
        robotHardware.backboardCamera.setPipeline(pipeline);

        robotHardware.backboardCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotHardware.backboardCamera.startStreaming(constants.cameraWidth, constants.cameraHeight, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    public void eggs2() {
        if (op.gamepad2.a && !eggDown) {
            eggDown = true;
            eggPressed = !eggPressed;
        }
        else if (!op.gamepad2.a && eggDown)
            eggDown = false;
        if (eggPressed)
            robotHardware.eggsDown();
        else
            robotHardware.eggsUp();
    }
    public void eggs1() {
        if (op.gamepad1.x && !eggDown) {
            eggDown = true;
            eggPressed = !eggPressed;
        }
        else if (!op.gamepad1.x && eggDown)
            eggDown = false;
        if (eggPressed)
            robotHardware.eggsDown();
        else
            robotHardware.eggsUp();
    }

    public boolean getB() {
        return bPressed;
    }

}
