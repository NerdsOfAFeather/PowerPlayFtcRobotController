package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OldCode.TensorFlowObjectDetectionWebcam;

/** Created by Gavin */
@TeleOp(name="PowerPlayConfig", group="Linear Opmode")
@Disabled
public class PowerPlayConfig extends PowerPlayObjectDetection {

    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor liftLiftMotor = null;
    public Servo rightClawServo = null;
    public Servo leftClawServo = null;

    static final double AUTO_SPEED = 0.6;


    public void initDriveHardware() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        liftLiftMotor = hardwareMap.get(DcMotor.class, "LiftLiftMotor");
        rightClawServo = hardwareMap.get(Servo.class, "RightClawServo");
        leftClawServo = hardwareMap.get(Servo.class, "LeftClawServo");

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void driveForward(double seconds) {
        leftFrontDrive.setPower(AUTO_SPEED);
        leftBackDrive.setPower(AUTO_SPEED);
        rightFrontDrive.setPower(AUTO_SPEED);
        rightBackDrive.setPower(AUTO_SPEED);
        sleep((long) (seconds * 1000));
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(250);
    }

    public void driveBackwards(double seconds) {
        leftFrontDrive.setPower(-AUTO_SPEED);
        leftBackDrive.setPower(-AUTO_SPEED);
        rightFrontDrive.setPower(-AUTO_SPEED);
        rightBackDrive.setPower(-AUTO_SPEED);
        sleep((long) (seconds * 1000));
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(250);
    }

    public void driveRight(double seconds) {
        leftFrontDrive.setPower(AUTO_SPEED);
        leftBackDrive.setPower(-AUTO_SPEED);
        rightFrontDrive.setPower(-AUTO_SPEED);
        rightBackDrive.setPower(AUTO_SPEED);
        sleep((long) (seconds * 1000));
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(250);
    }

    public void driveLeft(double seconds) {
        leftFrontDrive.setPower(-AUTO_SPEED);
        leftBackDrive.setPower(AUTO_SPEED);
        rightFrontDrive.setPower(AUTO_SPEED);
        rightBackDrive.setPower(-AUTO_SPEED);
        sleep((long) (seconds * 1000));
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(250);
    }

    public void turnRight(double seconds) {
        leftFrontDrive.setPower(AUTO_SPEED);
        leftBackDrive.setPower(AUTO_SPEED);
        rightFrontDrive.setPower(-AUTO_SPEED);
        rightBackDrive.setPower(-AUTO_SPEED);
        sleep((long) (seconds * 1000));
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(250);
    }

    public void turnLeft(double seconds) {
        leftFrontDrive.setPower(-AUTO_SPEED);
        leftBackDrive.setPower(-AUTO_SPEED);
        rightFrontDrive.setPower(AUTO_SPEED);
        rightBackDrive.setPower(AUTO_SPEED);
        sleep((long) (seconds * 1000));
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(250);
    }

    public void liftUp(double seconds){
        liftLiftMotor.setPower(0.5);
        sleep((long) (seconds * 1000));
        liftLiftMotor.setPower(0);
        sleep(250);
    }

    public void liftDown(double seconds){
        liftLiftMotor.setPower(-0.5);
        sleep((long) (seconds * 1000));
        liftLiftMotor.setPower(0);
        sleep(250);
    }

    public void clampClose(){
        leftClawServo.setPosition(1.0);
        rightClawServo.setPosition(0.0);
    }

    public void clampOpen(){
        leftClawServo.setPosition(0.0);
        rightClawServo.setPosition(1.0);
    }

}
