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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is sample code used to explain how to write an autonomous code
 *
 */
// Starting at the edge of the blue depot

@Autonomous(name="Blue skystone", group="Pushbot")
//@Disabled
public class blueVuforiaSkystone extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2019 robot = new HardwareJoeBot2019();   // Use a Pushbot's hardware
    Utility13702 U = new Utility13702();
    Image_Recognition I = new Image_Recognition();

    double xValue;
    double yValue;


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry.addLine("Press > to Start");
        telemetry.update();

        robot.init(hardwareMap, this);
        U.init(hardwareMap, this);
        I.init(hardwareMap,this);

        waitForStart();

        robot.moveInches(6.3, 0.42, 10);
        sleep(300);
        //move all mechanisms out
        U.leftIntakeServoOut();
        sleep(300);


        //move lift up
        U.moveLiftEncoder(-900);
        sleep(1000);

        //move arm out
        U.moveArmEncoder(U.ARM_AUTO_GRABBING);
        sleep(1000);

        U.clampClosedHorizontal();
        sleep(300);


        //variable for coordinates
        double coords[] = {777,777};

        //loop over I.skystone coordiates a few times
        int i = 0;
        while (i < 25) {
            coords = I.skystone_cooridinates();
            i = i + 1;
            sleep(80);
        }

        //get the second coordinate
        //coords[1];
        // if the second coordinate is less than 0, position 1
        //if it's greateer than 0, position 2
        //if it's not found (777), position 3

        if(coords[1] < 0){
            telemetry.addLine("first skystone seen");
            telemetry.update();

            robot.moveInches(18,0.28,10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(500);
 
            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(900);

            robot.moveInches(-15, -.27,10);
            robot.resetDegrees(0.15);
            //robot.strafeSeconds(1700, -0.25);
            //robot.resetDegrees(0.15);
            robot.rotateDegrees(-85,0.25);
            robot.moveInches(21, 0.4, 10);
            sleep(800);
            robot.rotateDegrees(85, 0.25);

            //move to foundation
            robot.moveInches(15,0.4, 15);
            sleep(300);

        }else if(coords[1] != 777){
            telemetry.addLine("second skystone seen");
            telemetry.update();

            robot.strafeSeconds(500,0.25);
            robot.resetDegrees(0.25);
            robot.moveInches(19,0.28,10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(1400);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(1000);

            robot.moveInches(-15.5, -.25,10);
            //robot.strafeSeconds(2300, -0.25);
            //robot.resetDegrees(0.15);

            robot.rotateDegrees(-85,0.25);
            robot.moveInches(28, 0.4, 10);
            sleep(1700);
            robot.rotateDegrees(85, 0.25);

            //move to foundation
            robot.moveInches(13.5, 0.5, 10);
            sleep(300);

        }else{
            telemetry.addLine("third skystone seen");
            telemetry.update();

            robot.strafeSeconds(800, 0.25);
            robot.moveInches(13, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(1400);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(1000);

            robot.moveInches(-15, -.25, 10);
            //robot.strafeSeconds(2800, -0.25);
            //robot.resetDegrees(0.25);

            robot.rotateDegrees(-85,0.25);
            robot.moveInches(30, 0.4, 10);
            sleep(1300);
            robot.rotateDegrees(85, 0.25);

            robot.moveInches(16, 0.5, 10);
            sleep(300);

        }


        robot.strafeSeconds(300,-0.7);

        U.moveArmEncoder(U.ARM_OUT_POSITION);
        U.moveLiftEncoder(U.LIFT_UP_POSITION);

        sleep(1500);
        //grab foundation
        U.closeGrabber();

        sleep(500);
        //drive into building site
        U.clampVertical();
        U.leftIntakeServoOut();
        U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
        U.moveArmEncoder(U.ARM_IN_POSITION);
        robot.moveInches(-90, 0.25,15);

        robot.strafeSeconds(1500, 0.5);

        // robot.moveInches(-10, 0.25, 10);

        //release grabber
        U.openGrabber();

        sleep(500);


        //back up under skybridge
        robot.moveInches(20,0.25,10);
        robot.strafeSeconds(1100, -0.25);
        robot.moveInches(28, 0.25, 10);

    }
}
