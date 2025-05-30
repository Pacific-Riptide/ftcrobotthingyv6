package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class ScoreSampleBackAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;
    Servo claw360;

    public ScoreSampleBackAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota, Servo claw360)
    {
        this.armMotor = armMotor;
        this.rotaMotor = rotaMotor;
        this.clawServo = clawServo;
        this.clawRota = clawRota;
        this.claw360 = claw360;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT, false);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_TO_SAMPLE_BASKET_HEIGHT, true);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_BACK, false);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_UP);
            Thread.sleep(1000);
            clawServo.setPosition(RobotConstants.OPEN_CLAW);
            Thread.sleep(300);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);
            DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_DOWN, true);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        return false;
    }
}
