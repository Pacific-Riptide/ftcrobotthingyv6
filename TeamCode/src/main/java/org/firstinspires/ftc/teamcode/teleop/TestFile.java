package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.actions.TeleOpScoreSampleBackAction;
import org.firstinspires.ftc.teamcode.auto.actions.TeleOpSpecimenPickupAction;
import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

@TeleOp
public final class TestFile extends LinearOpMode {

    //Start
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
            displayTelemetry();
        }
    }

    public void displayTelemetry() {
        telemetry.addData("A Button", gamepad1.a);
        telemetry.addData("B Button", gamepad1.b);
        telemetry.addData("X Button", gamepad1.x);
        telemetry.addData("Y Button", gamepad1.y);
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Dpad Up", gamepad1.dpad_up);
        telemetry.addData("Dpad Down", gamepad1.dpad_down);
        telemetry.addData("Dpad Left", gamepad1.dpad_left);
        telemetry.addData("Dpad Right", gamepad1.dpad_right);

        // Add telemetry for other buttons or joystick axes
        telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
        telemetry.addData("Right Stick Button", gamepad1.right_stick_button);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);

        telemetry.addData("Start", gamepad1.start);
        telemetry.addData("Share", gamepad1.share);
        telemetry.addData("Back", gamepad1.back);
        telemetry.addData("Guide", gamepad1.guide);
        telemetry.addData("Options", gamepad1.options);
        telemetry.addData("PS", gamepad1.ps);
        telemetry.addData("Touchpad", gamepad1.touchpad);
        telemetry.addData("Finger 1", gamepad1.touchpad_finger_1);
        telemetry.addData("Finger 2", gamepad1.touchpad_finger_2);

        telemetry.update();
    }


}

