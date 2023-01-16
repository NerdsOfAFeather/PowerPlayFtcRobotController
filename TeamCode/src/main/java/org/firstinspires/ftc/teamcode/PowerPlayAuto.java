/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/*
  This file illustrates the concept of driving a path based on time.
  The code is structured as a LinearOpMode
 */

/** Created by Gavin */
@Autonomous(name = "Robot: Auto Drive By Time", group = "Robot")
public class PowerPlayAuto extends PowerPlayConfig {

    /* Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor liftLiftMotor = null;
    public Servo rightClawServo = null;
    public Servo leftClawServo = null;

    static final double SPEED = 0.6;

     */
    @Override
    public void runOpMode() {

        /*
        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        liftLiftMotor = hardwareMap.get(DcMotor.class, "LiftLiftMotor");
        rightClawServo = hardwareMap.get(Servo.class, "RightClawServo");
        leftClawServo = hardwareMap.get(Servo.class, "LeftClawServo");

        // To drive forward, most robots need the motor on one side to be reversed,
        // because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust
        // these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear
        // Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

         */

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        initDriveHardware();
        initVuforia();
        initTfod();

        if (tfod != null){
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 1) {
                    Recognition r = updatedRecognitions.get(0);
                    telemetry.addData("Object Detected: ", r.getLabel());
                    telemetry.update();
                    chooseProgram();
                } else {
                    telemetry.addData("Status:", "Confused");
                    telemetry.update();
                }
            }
        }
    }

    public void chooseProgram(){
        while (!isStarted()) {
            telemetry.addData("Please select a Auto to run", "");
            telemetry.addData( "A - Blue Left | B - Blue Right | X - Red Left | Y - Red Right", "");
            telemetry.update();
            if (gamepad1.a) {
                telemetry.addData("RobotPosition", "Blue Left");
                blueLeft();
                telemetry.update();
                break;
            } else if (gamepad1.b) {
                telemetry.addData("RobotPosition", "Blue Right");
                blueRight();
                telemetry.update();
                break;
            } else if (gamepad1.x) {
                telemetry.addData("RobotPosition", "Red Left");
                redLeft();
                telemetry.update();
                break;
            } else if (gamepad1.y) {
                telemetry.addData("RobotPosition", "Red Right");
                redRight();
                telemetry.update();
                break;
            } else {
                telemetry.addData("waiting", "help");
                telemetry.update();
            }
        }
    }

    public void blueLeft(){
        waitForStart();
        if (getRecognition().equals("one")){
            telemetry.addData("Running Code: ", "Blue Left One");
            telemetry.update();
        } else if (getRecognition().equals("two")){
            telemetry.addData("Running Code: ", "Blue Left Two");
            telemetry.update();
        } else if (getRecognition().equals("three")){
            telemetry.addData("Running Code: ", "Blue Left Three");
            telemetry.update();
        }
    }

    public void blueRight(){
        waitForStart();
        if (getRecognition().equals("one")){
            telemetry.addData("Running Code: ", "Blue Right One");
            telemetry.update();
        } else if (getRecognition().equals("two")){
            telemetry.addData("Running Code: ", "Blue Right Two");
            telemetry.update();
        } else if (getRecognition().equals("three")){
            telemetry.addData("Running Code: ", "Blue Right Three");
            telemetry.update();
        }
    }

    public void redLeft(){
        waitForStart();
        if (getRecognition().equals("one")){
            telemetry.addData("Running Code: ", "Red Left One");
            telemetry.update();
        } else if (getRecognition().equals("two")){
            telemetry.addData("Running Code: ", "Red Left Two");
            telemetry.update();
        } else if (getRecognition().equals("three")){
            telemetry.addData("Running Code: ", "Red Left Three");
            telemetry.update();
        }
    }

    public void redRight(){
        waitForStart();
        if (getRecognition().equals("one")){
            telemetry.addData("Running Code: ", "Red Right One");
            telemetry.update();
        } else if (getRecognition().equals("two")){
            telemetry.addData("Running Code: ", "Red Right Two");
            telemetry.update();
        } else if (getRecognition().equals("three")){
            telemetry.addData("Running Code: ", "Red Right Three");
            telemetry.update();
        }
    }

    /*
    public void driveForward(double seconds) {
        leftFrontDrive.setPower(SPEED);
        leftBackDrive.setPower(SPEED);
        rightFrontDrive.setPower(SPEED);
        rightBackDrive.setPower(SPEED);
        sleep((long) (seconds * 1000));
		leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
		sleep(250);
    }

    public void driveBackwards(double seconds) {
        leftFrontDrive.setPower(-SPEED);
        leftBackDrive.setPower(-SPEED);
        rightFrontDrive.setPower(-SPEED);
        rightBackDrive.setPower(-SPEED);
        sleep((long) (seconds * 1000));
		leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
		sleep(250);
    }

    public void driveRight(double seconds) {
        leftFrontDrive.setPower(SPEED);
        leftBackDrive.setPower(-SPEED);
        rightFrontDrive.setPower(-SPEED);
        rightBackDrive.setPower(SPEED);
        sleep((long) (seconds * 1000));
		leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
		sleep(250);
    }

	public void driveLeft(double seconds) {
        leftFrontDrive.setPower(-SPEED);
        leftBackDrive.setPower(SPEED);
        rightFrontDrive.setPower(SPEED);
        rightBackDrive.setPower(-SPEED);
        sleep((long) (seconds * 1000));
		leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
		sleep(250);
    }

    public void turnRight(double seconds) {
        leftFrontDrive.setPower(SPEED);
        leftBackDrive.setPower(SPEED);
        rightFrontDrive.setPower(-SPEED);
        rightBackDrive.setPower(-SPEED);
        sleep((long) (seconds * 1000));
		leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
		sleep(250);
    }

	public void turnLeft(double seconds) {
        leftFrontDrive.setPower(-SPEED);
        leftBackDrive.setPower(-SPEED);
        rightFrontDrive.setPower(SPEED);
        rightBackDrive.setPower(SPEED);
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
	*/
}