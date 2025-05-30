package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class TeleOpScoreSampleBackAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;
    Servo claw360;

    public TeleOpScoreSampleBackAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota, Servo claw360)
    {
        this.armMotor = armMotor;
        this.rotaMotor = rotaMotor;
        this.clawServo = clawServo;
        this.clawRota = clawRota;
        this.claw360 = claw360;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
        DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT - 100, false);
        DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_TO_MAX_HEIGHT, true);
        clawRota.setPosition(RobotConstants.CLAW_ROTA_UP);

        return false;
    }
}
