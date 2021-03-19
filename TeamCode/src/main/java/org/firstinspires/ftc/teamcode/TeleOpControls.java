package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeleOpControls {
    LinearOpMode op;
    RobotHardware robotHardware;
    Telemetry telemetry;
    boolean aPressed = false;
    boolean aDown = false;

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
        robotHardware.intake.setPower(op.gamepad1.right_stick_y);
        robotHardware.cleanser.setPower((op.gamepad1.right_bumper ? 1 : 0) + (op.gamepad1.right_stick_y * 0.25));

        //shooter
        //  A toggle on/off
        if (op.gamepad1.a && !aDown) {
            aDown = true;
            aPressed = !aPressed;
        }
        else if (!op.gamepad1.a && aDown) {
            aDown = false;
        }
        robotHardware.shooter.setVelocity(aPressed ? 1500 : 0);

    }

}
