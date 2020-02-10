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

@Autonomous(name="Red Corner Skystone", group="Pushbot")
@Disabled
public class redCornerSkystone extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2019      robot   = new HardwareJoeBot2019();   // Use a Pushbot's hardware
    Utility13702        U = new Utility13702();
    Image_Recognition    I = new Image_Recognition();
    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry.addLine("Press > to Start");
        telemetry.update();

        robot.init(hardwareMap,this);
        U.init(hardwareMap, this);
        I.init(hardwareMap,this);


        waitForStart();

        robot.moveInches(6, 0.42, 10);
        //////////////////


        //move all mechanisms out
        U.leftIntakeServoOut();

        //move lift up
        U.moveLiftEncoder(-900);
        sleep(300);

        //move arm out
        U.moveArmEncoder(U.ARM_AUTO_GRABBING);
        U.grabBlock();
        sleep(200);

        U.leftIntakeServo.setPosition(U.LEFT_INTAKE_SERVO_AUTO_POSITION);

        //variable for coordinates
        double coords[] = {777, 777};

        //loop over I.skystone coordiates a few times
        int i = 0;
        while (i < 15) {
            coords = I.skystone_cooridinates();
            i = i + 1;
            sleep(80);
        }


        if (coords[1] < 0) {

            robot.moveInches(16, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(500);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(900);

            robot.moveInches(-45, .25, 10);
            robot.rotateDegrees(85, 0.25);
            robot.strafeSeconds(1000,0.25);
            robot.moveInches(85, 0.4, 10);
            sleep(100);

            U.moveArmEncoder(U.ARM_OUT_POSITION);
            robot.moveInches(-16,0.3,10);

            U.clampVertical();
            U.leftIntakeServoOut();

        } else if(coords[1] != 777){

            //SECOND SKYSTONE POS
            robot.strafeSeconds(500, 0.25);
            robot.moveInches(24, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(500);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(900);

            robot.moveInches(-45, .25, 10);
            robot.rotateDegrees(85, 0.25);
            robot.strafeSeconds(1000,0.25);
            robot.moveInches(75, 0.4, 10);
            sleep(100);

            U.moveArmEncoder(U.ARM_OUT_POSITION);
            robot.moveInches(-21 ,0.5, 10);

            U.clampVertical();
            U.leftIntakeServoOut();

        }else{

            //THIRD SKYSTONE POS
            robot.strafeSeconds(900, 0.2);

            U.leftIntakeServoIn();

            robot.moveInches(24, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(500);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(900);

            robot.moveInches(-45, .25, 10);
            robot.rotateDegrees(85, 0.25);
            robot.strafeSeconds(1000,0.25);
            robot.moveInches(67, 0.4, 10);
            sleep(100);

            U.leftIntakeServoOut();
            U.moveArmEncoder(U.ARM_OUT_POSITION);

            robot.moveInches(-7,0.5, 10);
        }

        stop();

    }

}
