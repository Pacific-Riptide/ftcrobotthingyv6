package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.jar.Attributes;

@TeleOp
public class ServoTest extends OpMode {

    private CRServo actuator;

    @Override
    public void init() {
        actuator = hardwareMap.crservo.get("actuatorServo");

    }

    @Override
    public void loop() {

        if (gamepad1.y){
            actuator.setPower(1);
        }
        actuator.setPower(0);

    }
}
