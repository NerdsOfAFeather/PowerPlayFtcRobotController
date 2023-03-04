package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

/** Created by Gavin for Team 6347*/
@TeleOp(name="OldPowerPlayConfig", group="Linear Opmode")
@Disabled
public class OldPowerPlayConfig extends PowerPlayObjectDetection {

    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor liftLiftMotor = null;
    public Servo rightClawServo = null;
    public Servo leftClawServo = null;
    public ColorSensor color;
    public IMU imu;

    public static int desiredLiftPosition = -2;
    public static boolean liftMoving = false;

    static final int lvl0 = 0;
    static final int lvl1 = -2500;
    static final int lvl2 = -4250;
    static final int lvl3 = -5850;

    static final double AUTO_SPEED = 0.6;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;


    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

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
        color = hardwareMap.get(ColorSensor.class, "Color");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        liftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initLift(){
        liftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        liftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLiftMotor.setPower(-0.5);
        sleep((long) (seconds * 1000));
        liftLiftMotor.setPower(0);
        sleep(250);
        liftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftDown(double seconds){
        if (seconds != 0) {
            liftLiftMotor.setPower(0.5);
            sleep((long) (seconds * 1000));
            liftLiftMotor.setPower(0);
            sleep(250);
        }
    }

    public void clampClose(){
        leftClawServo.setPosition(0.0);
        rightClawServo.setPosition(1.0);
        sleep(500);
    }

    public void clampOpen(){
        leftClawServo.setPosition(1.0);
        rightClawServo.setPosition(0.0);
        sleep(500);
    }

    public void driveForwardSlow(double seconds){
        leftFrontDrive.setPower(AUTO_SPEED/2);
        leftBackDrive.setPower(AUTO_SPEED/2);
        rightFrontDrive.setPower(AUTO_SPEED/2);
        rightBackDrive.setPower(AUTO_SPEED/2);
        sleep((long) (seconds * 1000));
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(250);
    }

    public void driveBackwardSlow(double seconds){
        leftFrontDrive.setPower(-AUTO_SPEED/2);
        leftBackDrive.setPower(-AUTO_SPEED/2);
        rightFrontDrive.setPower(-AUTO_SPEED/2);
        rightBackDrive.setPower(-AUTO_SPEED/2);
        sleep((long) (seconds * 1000));
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(250);
    }

    @Deprecated
    public int getDesiredLocationColor() {
        int max = Math.max(Math.max(color.red(), color.blue()), color.green());
        int red = color.red();
        int green = color.green();
        int blue = color.blue();
        if (max == red) { //Red?
            return 1;
        } else if (max == green) { // Green?
            return 2;
        } else { //Blue?
            return 3;
        }
    }


}
