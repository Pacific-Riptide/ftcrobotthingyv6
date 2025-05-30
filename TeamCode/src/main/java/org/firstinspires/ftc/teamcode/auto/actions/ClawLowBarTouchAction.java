package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class ClawLowBarTouchAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;

    public ClawLowBarTouchAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota)
    {
        this.armMotor = armMotor;
        this.rotaMotor = rotaMotor;
        this.clawServo = clawServo;
        this.clawRota = clawRota;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {

            DCMotorUtils.controlMotorWithPID(armMotor, 300, true);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_UP);
            clawServo.setPosition(RobotConstants.CLOSED_CLAW);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(rotaMotor, 200, false);
            Thread.sleep(100);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);
        }
        catch (InterruptedException e) {
        }


        return false;
    }
}
