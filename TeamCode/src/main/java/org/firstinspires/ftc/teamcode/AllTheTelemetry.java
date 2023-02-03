package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Telemetry")
public class AllTheTelemetry extends PowerPlayConfig{

    @Override
    public void runOpMode() {

        while (opModeIsActive()) {
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);
            telemetry.addData("Left Claw Position", leftClawServo.getPosition());
            telemetry.addData("Right Claw Position", rightClawServo.getPosition());
            telemetry.addLine("Left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", gamepad1.left_stick_y);
            telemetry.addLine("Light joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);
            telemetry.addLine("Color sensor | ")
                    .addData("Red", color.red())
                    .addData("Green", color.green())
                    .addData("Blue", color.blue())
                    .addData("ARGB", color.argb());
            telemetry.addLine("Magnetic Limit Switches")
                    .addData("1", limit1.getValue())
                    .addData("2", limit2.getValue());
            telemetry.update();
        }
    }
}
