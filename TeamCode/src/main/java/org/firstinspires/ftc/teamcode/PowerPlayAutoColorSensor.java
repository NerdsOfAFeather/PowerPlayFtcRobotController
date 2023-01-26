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

/* This file illustrates the concept of driving a path based on time.
   The code is structured as a LinearOpMode
 */

/** Created by Gavin */
@Autonomous(name = "Power Play Auto No TFOD", group = "Auto")
public class PowerPlayAutoColorSensor extends PowerPlayConfig {

    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        initDriveHardware();
        chooseProgram();
    }

    public void chooseProgram(){
        while (!isStarted()) {
            telemetry.addData("Please select a Auto to run", "");
            telemetry.addData( "A - Blue Left | B - Blue Right | X - Red Left | Y - Red Right", "");
            telemetry.update();
            if (gamepad1.a) {
                telemetry.addData("RobotPosition", "Blue Left");
                telemetry.update();
                blueLeft();
                break;
            } else if (gamepad1.b) {
                telemetry.addData("RobotPosition", "Blue Right");
                telemetry.update();
                blueRight();
                break;
            } else if (gamepad1.x) {
                telemetry.addData("RobotPosition", "Red Left");
                telemetry.update();
                redLeft();
                break;
            } else if (gamepad1.y) {
                telemetry.addData("RobotPosition", "Red Right");
                telemetry.update();
                redRight();
                break;
            } else {
                telemetry.addData("waiting", "help");
                telemetry.update();
            }
        }
    }

    public void blueLeft(){
        waitForStart();

    }

    public void blueRight(){
        waitForStart();

    }

    public void redLeft(){
        waitForStart();
        clampClose();
        liftUp(1);
        sleep(250);
        driveForward(2.5);
        //int red = color.red();
        //int blue = color.blue();
        //int green = color.green();
        driveForward(.5);  //Push the cone
        driveBackwards(.5);
        driveRight(2.5);
        liftUp(10); //All the way
        driveRight(1);
        clampOpen();
        driveLeft(7);
        driveForward(1);
        //liftDown(3);
        /*
        if (red >= .5) { // Position 1?
            driveBackwards(.5);
            driveRight(2);
            liftUp(3); //All the way
            driveForward(.25);
            clampOpen();
            driveBackwards(.25);
            liftDown(3);
            driveLeft(4);
        } else if (blue >= .5) { // Position 2?
            driveBackwards(.5);//out of the way
            driveRight(2);
            liftUp(3); //All the way
            driveForward(.25);
            clampOpen();
            driveBackwards(.25);
            liftDown(3);
            driveLeft(3);
        } else if (green >= .5) { // Position 3?
            driveBackwards(.5);
            driveRight(2);
            liftUp(3); //All the way
            driveForward(.25);
            clampOpen();
            driveBackwards(.25);
            liftDown(3);
            driveLeft(2);
        } else {
            driveBackwards(.5);
            driveRight(2);
            liftUp(3); //All the way
            driveForward(.25);
            clampOpen();
            driveBackwards(.25);
            liftDown(3);
            driveLeft(3);
        }
        */

    }

    public void redRight(){
        waitForStart();

    }
}