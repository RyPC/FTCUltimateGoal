package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeleOpControls {
    LinearOpMode op;
    RobotHardware robotHardware;
    Telemetry telemetry;
    Constants constants = new Constants();
    int shooterSpeed = constants.shooterPower;
    boolean aPressed = false;
    boolean aDown = false;
    boolean rbPressed = false;
    boolean rbDown = false;
    boolean lbPressed = true;
    boolean lbDown = false;
    boolean bPressed = false;
    boolean bDown = false;


    public TeleOpControls(LinearOpMode op, RobotHardware robotHardware, Telemetry telemetry) {
        this.op = op;
        this.robotHardware = robotHardware;
        this.telemetry = telemetry;
    }

    public void allControlsJustToSpiteGiebe () {
        //movement
        //  left stick: strafing/movement
        //  triggers: rotation
        //  A toggle slow mode: 50% speed
        double forward = -op.gamepad1.left_stick_y;
        double strafe = op.gamepad1.left_stick_x;
        double rotate = op.gamepad1.right_trigger - op.gamepad1.left_trigger;

        robotHardware.fr.setPower((aPressed ? 0.375 : 0.75) * (forward - rotate - strafe));
        robotHardware.fl.setPower((aPressed ? 0.375 : 0.75) * (forward + rotate + strafe));
        robotHardware.br.setPower((aPressed ? 0.375 : 0.75) * (forward - rotate + strafe));
        robotHardware.bl.setPower((aPressed ? 0.375 : 0.75) * (forward + rotate - strafe));

        //intake/cleanser
        //  right stick
        robotHardware.intake.setPower(op.gamepad1.a ? -0.5 : op.gamepad1.right_stick_y);
        robotHardware.cleanser.setPower( op.gamepad1.a ? -1 :
                (aPressed && Math.abs(constants.shooterPower - robotHardware.shooter.getVelocity()) < 75) || !aPressed ?
                op.gamepad1.right_stick_y * 0.5 : 0);

        //shooter
        //  A toggle on/off
        if (op.gamepad1.a && !aDown) {
            aDown = true;
            aPressed = !aPressed;
        }
        else if (!op.gamepad1.a && aDown) {
            aDown = false;
        }
        robotHardware.shooter.setVelocity((aPressed && !op.gamepad1.a) ? shooterSpeed : 0);

        //wobble goal
        if (op.gamepad1.right_bumper && !rbDown) {
            rbDown = true;
            rbPressed = !rbPressed;
        }
        else if (!op.gamepad1.right_bumper && rbDown) {
            rbDown = false;
        }
        robotHardware.wobbleClamp.setPosition(rbPressed ? constants.clampDown : constants.clampUp);

        if (op.gamepad1.left_bumper && !lbDown) {
            lbDown = true;
            lbPressed = !lbPressed;
        }
        else if (!op.gamepad1.left_bumper && lbDown) {
            lbDown = false;
        }
        robotHardware.wobbleArm.setPosition(lbPressed ? constants.armUp : constants.armDown);

        //power shots mode
        if (op.gamepad1.b && !bDown) {
            bDown = true;
            bPressed = !bPressed;
        }
        else if (!op.gamepad1.b && bDown) {
            bDown = false;
        }

        shooterSpeed = bPressed ? constants.shooterPower - 100 : constants.shooterPower;

    }
//    public boolean getBPressed() {
//        return bPressed;
//    }

}
