package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class SampleDropAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;

    int armHeight;

    public SampleDropAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota)
    {
        this.armMotor = armMotor;
        this.rotaMotor = rotaMotor;
        this.clawServo = clawServo;
        this.clawRota = clawRota;
        this.armHeight = armHeight;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(armMotor, 100, true);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_DOWN, false);
            Thread.sleep(100);
            clawServo.setPosition(RobotConstants.OPEN_CLAW);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT, false);
        }
        catch (InterruptedException e)
        {

        }

        return false;
    }
}
