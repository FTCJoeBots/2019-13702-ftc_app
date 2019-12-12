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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is sample code used to explain how to write an autonomous code
 *
 */

@Autonomous(name="Red Everything", group="Pushbot")
//@Disabled
public class redEverything extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2019      robot   = new HardwareJoeBot2019();   // Use a Pushbot's hardware
    Utility13702      U   = new Utility13702();
    Image_Recognition    I = new Image_Recognition();
    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry.addLine("Press > to Start");
        telemetry.update();

        robot.init(hardwareMap,this);
        U.init(hardwareMap,this);
        I.init(hardwareMap,this);
        waitForStart();

        //move to foundation
        robot.moveInches(-40,0.25, 10);
        sleep(300);
        robot.strafeSeconds(640,-0.7);

        //grab foundation
        U.closeGrabber();
        sleep(500);

        //drive into building site
        robot.moveInches(90, 0.25,15);
        robot.strafeSeconds(1000,0.5);
        // robot.moveInches(20, 0.25, 10);

        //release grabber
        U.openGrabber();
        sleep(500);

        //back up under skybridge
        robot.moveInches(-100,0.25,10);
        robot.rotateDegrees(-85, .5);
        robot.moveInches(5, 0.1, 10);
        robot.strafeSeconds(1000,-.3);
        robot.strafeSeconds(250,0.3);

        robot.moveInches(-6.3, 0.42, 10);
        sleep(300);

        //delay////////////
        sleep(0);

        //////////////////

        robot.moveInches(6.8, 0.45, 10);
        sleep(300);

        //move all mechanisms out
        U.leftIntakeServoOut();

        //move lift up
        U.moveLiftEncoder(-900);
        sleep(500);

        //move arm out
        U.moveArmEncoder(U.ARM_AUTO_GRABBING);
        U.clampClosedHorizontal();

        U.leftIntakeServo.setPosition(U.LEFT_INTAKE_SERVO_AUTO_POSITION);
        sleep(500);

        //variable for coordinates
        double coords[] = {777, 777};

        //loop over I.skystone coordiates a few times
        int i = 0;
        while (i < 40) {
            coords = I.skystone_cooridinates();
            i = i + 1;
            sleep(80);
        }


        if (coords[1] > 0 && coords[1] != 777) {
            robot.strafeSeconds(650, 0.25);

            robot.moveInches(22, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(500);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(900);

            robot.moveInches(-19, .25, 10);
            //robot.strafeSeconds(1800, 0.25);
            //robot.resetDegrees(0.15);
            robot.rotateDegrees(85, 0.25);
            robot.moveInches(37, 0.25, 10);
            sleep(100);

            U.moveArmEncoder(U.ARM_AUTO_RELEASE_BLOCK);
            U.moveLiftEncoder(U.LIFT_AUTO_RELEASE_BLOCK);
            sleep(1500);

            robot.moveInches(-7,0.3, 10);

            U.clampVertical();
            U.leftIntakeServoOut();
            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            U.moveArmEncoder(U.ARM_IN_POSITION);
            sleep(1500);

            robot.moveInches(-7, 0.3, 10);
            robot.strafeSeconds(1000, -0.25);

        } else if(coords[1] < 0){

            //SECOND SKYSTONE POS
            robot.strafeSeconds(200, 0.25);
            robot.moveInches(24, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(500);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(900);

            robot.moveInches(-20, .25, 10);
            //robot.strafeSeconds(1800, 0.25);
            //robot.resetDegrees(0.15);
            robot.resetDegrees(0.15);
            robot.rotateDegrees(85, 0.25);
            robot.moveInches(50, 0.25, 10);
            sleep(100);

            U.moveArmEncoder(U.ARM_AUTO_RELEASE_BLOCK);
            U.moveLiftEncoder(U.LIFT_AUTO_RELEASE_BLOCK);
            sleep(1500);

            robot.moveInches(-8 ,0.5, 10);
            U.clampVertical();
            U.leftIntakeServoOut();
            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            U.moveArmEncoder(U.ARM_IN_POSITION);
            sleep(1500);

            robot.moveInches(-9, 0.5, 10);
            robot.strafeSeconds(1000, -0.25);

        }else{

            //THIRD SKYSTONE POS
            robot.strafeSeconds(200, -0.25);

            U.leftIntakeServoIn();

            robot.moveInches(24, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(500);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(900);

            robot.moveInches(-21, .25, 10);
            //robot.strafeSeconds(1800, 0.25);
            //robot.resetDegrees(0.15);
            robot.resetDegrees(0.15);
            robot.rotateDegrees(85, 0.25);
            robot.moveInches(62, 0.25, 10);
            sleep(100);

            U.leftIntakeServoOut();
            U.moveArmEncoder(U.ARM_AUTO_RELEASE_BLOCK);
            U.moveLiftEncoder(U.LIFT_AUTO_RELEASE_BLOCK);
            sleep(1500);

            robot.moveInches(-13,0.5, 10);
            U.clampVertical();
            U.leftIntakeServoOut();
            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            U.moveArmEncoder(U.ARM_IN_POSITION);
            sleep(1500);

            robot.moveInches(-10, 0.5,10);
            robot.strafeSeconds(1000, -0.25);
        }

        /*//move to foundation

        robot.moveInches(24,0.25, 15);
        U.moveArmEncoder(U.ARM_OUT_POSITION);
        U.moveLiftEncoder(U.LIFT_UP_POSITION);

        sleep(1500);

        U.clampVertical();
        U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
        U.moveArmEncoder(U.ARM_IN_POSITION);

        robot.rotateDegrees(175, 0.2);

        robot.strafeSeconds(640,-0.7);

        //grab foundation
        U.closeGrabber();

        sleep(1000);
        //drive into building site
        robot.moveInches(90, 0.25,15);

        robot.strafeSeconds(1500, 0.5);

        // robot.moveInches(-10, 0.25, 10);

        //release grabber
        U.openGrabber();

        sleep(1000);


        //back up under skybridge
        robot.moveInches(20,-0.25,10);
        robot.strafeSeconds(1400, -0.25);
        robot.moveInches(25, -0.25, 10);



        telemetry.addLine("done");
        telemetry.update();*/


        telemetry.addLine("We're done. Press stop.");
        telemetry.update();


        telemetry.addLine("We're done. Press stop.");
        telemetry.update();

    }

}


