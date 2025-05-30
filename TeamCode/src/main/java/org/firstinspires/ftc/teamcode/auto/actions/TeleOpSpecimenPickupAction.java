package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class TeleOpSpecimenPickupAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;
    Servo claw360;
    int armHeight;

    public TeleOpSpecimenPickupAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota, Servo claw360)
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

            //When getting robot do this
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_SPECIMEN_PICK);
            clawServo.setPosition(RobotConstants.OPEN_CLAW);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_SPECIMEN_PICK + 200, false);
            Thread.sleep(100);
            //DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_INCOMING, true);
        }
        catch (InterruptedException e)
        {

        }

        return false;
    }
}
