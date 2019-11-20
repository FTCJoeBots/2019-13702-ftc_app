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

        double coords[] = I.skystone_cooridinates();
        ///Distance from skystone
        ///    coords[0]
        //Amount off center of skystone
        ///    coords[1]

        //drive to detect skystone
        robot.moveInches(13,0.15,10);
        robot.strafeSeconds(300, 0.25);
        sleep(500);


        while (coords[0] == 777) {
            robot.strafeSeconds(300, -0.25);

            sleep(500);


            coords = I.skystone_cooridinates();
        }
        telemetry.addData("done first while, sleep", coords[1]/22.4);
        telemetry.update();

        //found skystone, centering onto it

        coords = I.skystone_cooridinates();
            yValue = coords[1]/22.4;
            xValue = coords[0]/22.4;

            while(coords[1] < 5){
                telemetry.addData("second while loop", coords[1]);
                telemetry.update();

                robot.strafeSeconds(100,-0.25);
                robot.resetDegrees(0.3);

                sleep(400);
                coords = I.skystone_cooridinates();
            }

           /* U.closeClamp();
            robot.moveInches(20, .5, 5);
            robot.moveRobot(0, 27, 0);
            U.grabBlock();*/



    }
}
