package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class ScoreSampleFrontAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;

    public ScoreSampleFrontAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota)
    {
        this.armMotor = armMotor;
        this.rotaMotor = rotaMotor;
        this.clawServo = clawServo;
        this.clawRota = clawRota;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_TO_SAMPLE_BASKET_HEIGHT, true);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(rotaMotor, 400, false);
            clawRota.setPosition(0.3);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_TO_MAX_HEIGHT, true);
            Thread.sleep(100);
            clawServo.setPosition(RobotConstants.OPEN_CLAW);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT, false);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_DOWN, true);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        return false;
    }
}
