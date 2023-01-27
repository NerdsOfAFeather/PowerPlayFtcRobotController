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

/* This file illustrates the concept of driving a path based on time.
   The code is structured as a LinearOpMode
 */

/** Created by Gavin */
@Autonomous(name = "Power Play Auto TFOD", group = "Robot")
public class PowerPlayAuto extends PowerPlayConfig {

    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        initDriveHardware();
        //initVuforia();
        //initTfod();

        //Make a loop for tfod

        if (tfod != null){
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 1) {
                    // Send telemetry message to signify robot waiting
                    Recognition r = updatedRecognitions.get(0);
                    telemetry.addData("Status", "Ready to run");
                    telemetry.addData("Object Detected", r.getLabel());
                    telemetry.update();
                    chooseProgram();
                } else {
                    telemetry.addData("Status", "Confused");
                    telemetry.update();
                }
            }
        }
        waitForStart();
    }

    public void chooseProgram(){
        while (!isStarted()) {
            telemetry.addData("Please select an Auto to run", "");
            telemetry.addData( "A - Blue Left | B - Blue Right | X - Red Left | Y - Red Right", "");
            telemetry.update();
            if (gamepad1.a) {
                telemetry.addData("RobotPosition", "Blue Left");
                telemetry.update();
                sleep(500);
                blueLeft();
                break;
            } else if (gamepad1.b) {
                telemetry.addData("RobotPosition", "Blue Right");
                telemetry.update();
                sleep(500);
                blueRight();
                break;
            } else if (gamepad1.x) {
                telemetry.addData("RobotPosition", "Red Left");
                telemetry.update();
                sleep(500);
                redLeft();
                break;
            } else if (gamepad1.y) {
                telemetry.addData("RobotPosition", "Red Right");
                telemetry.update();
                sleep(500);
                redRight();
                break;
            } else {
                telemetry.addData("waiting", "help");
                telemetry.update();
                sleep(500);
            }
        }
    }

    public void blueLeft(){
        waitForStart();
        String rlabel = getRecognition();
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
}