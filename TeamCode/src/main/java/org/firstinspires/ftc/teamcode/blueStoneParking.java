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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is sample code used to explain how to write an autonomous code
 *
 */

@Autonomous(name="Blue Normal Stone + Foundation", group="Pushbot")
//@Disabled
public class blueStoneParking extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2019      robot   = new HardwareJoeBot2019();   // Use a Pushbot's hardware
    Utility13702      U   = new Utility13702();


    @Override
    public void runOpMode() {

        telemetry.addLine("Press > to Start");
        telemetry.update();

        robot.init(hardwareMap, this);
        U.init(hardwareMap,this);
        waitForStart();

        //move to foundation
        robot.moveInches(39, 0.25, 15);
        sleep(500);
        robot.strafeSeconds(640, -0.4);
        //grab foundation
        U.closeGrabber();

        sleep(500);
        //drive into building site
        robot.moveInches(-90, 0.35, 15);
        robot.strafeSeconds(1000, 0.5);
        // robot.moveInches(-10, 0.25, 10);

        //release grabber
        U.openGrabber();
        sleep(500);

        //goes to skystone cornner
        robot.moveInches(100, 0.57, 10);
        robot.rotateDegrees(-85, .5);
        robot.moveInches(-5, 0.2, 10);
        robot.strafeSeconds(1000,.3);

        robot.strafeSeconds(500,-0.2);
        robot.resetDegrees(0.15);
        sleep(100);
        //robot.moveInches(-5,0.1, 10);

        robot.moveInches(4, 0.25, 10);

        //move all mechanisms out
        U.leftIntakeServoOut();

        //move lift up
        U.moveLiftEncoder(-900);
        sleep(500);

        //move arm out
        U.moveArmEncoder(U.ARM_AUTO_GRABBING);
        U.grabBlock();
        sleep(250);

        U.leftIntakeServo.setPosition(U.LEFT_INTAKE_SERVO_AUTO_POSITION);
        sleep(500);

            robot.moveInches(18, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(500);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(900);

            robot.moveInches(-27, -.27, 10);
            //robot.strafeSeconds(1700, -0.25);
            //robot.resetDegrees(0.15);
            robot.rotateDegrees(-85, 0.25);
            robot.strafeSeconds(1000 ,-0.25);

            //goes forward beyond skybridge
            robot.moveInches(75, 0.4, 10);
            U.moveArmEncoder(U.ARM_OUT_POSITION);
            robot.moveInches(-14, 0.25, 10);

        }
    }

