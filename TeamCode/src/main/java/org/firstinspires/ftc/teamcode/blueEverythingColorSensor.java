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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * This is sample code used to explain how to write an autonomous code
 *
 */

@Autonomous(name="Blue Everything Color Sensor", group="Pushbot")
@Disabled
public class blueEverythingColorSensor extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2019 robot = new HardwareJoeBot2019();   // Use a Pushbot's hardware
    Utility13702Sensor U2 = new Utility13702Sensor();
    Utility13702 U = new Utility13702();
    Image_Recognition I = new Image_Recognition();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry.addLine("Press > to Start");
        telemetry.update();

        robot.init(hardwareMap, this);
        U2.init(hardwareMap, this);
        I.init(hardwareMap, this);

        // Get a reference to the color sensor
        U2.colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        U2.colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");

        // Get a reference to the distance sensor
        U2.distanceSensorRight = hardwareMap.get(DistanceSensor.class, "colorSensorRight");
        U2.distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "colorSensorLeft");

        // hsvValues is an array that will hold the hue, saturation, and value information
        float hsvValuesRight[] = {0F, 0F, 0F};
        float hsvValuesLeft[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array
        final float valuesRight[] = hsvValuesRight;
        final float valuesLeft[] = hsvValuesLeft;

        // Sometimes it helps to multiply the raw RGB values with a scale factor
        // To amplify/attentuate the measured values
        final double SCALE_FACTOR = 225;

        // Get a reference to the RelativeLayout so we can change the background
        // Color of the Robot Controller app to match the hue detected by the RGB sensor
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View reativLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        double SkystonePos = 50;

        waitForStart();

        // Loop and read the RGB and distance data
        // Note we use upModeIsActive() as our loop condition because it is an interruptible method

        // Convert the RGB values to HSV values
        // Multiply by the SCALE_FACTOR
        // Then cast is back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (U2.colorSensorRight.red() * SCALE_FACTOR),
                (int) (U2.colorSensorRight.green() * SCALE_FACTOR),
                (int) (U2.colorSensorRight.blue() * SCALE_FACTOR),
                hsvValuesRight);

        Color.RGBToHSV((int) (U2.colorSensorLeft.red() * SCALE_FACTOR),
                (int) (U2.colorSensorLeft.green() * SCALE_FACTOR),
                (int) (U2.colorSensorLeft.blue() * SCALE_FACTOR),
                hsvValuesLeft);

        // Send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm) Right",
                String.format(Locale.US, "%.02f", U2.distanceSensorRight.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha Right", U2.colorSensorRight.alpha());
        telemetry.addData("Red Right", U2.colorSensorRight.red());
        telemetry.addData("Green Right", U2.colorSensorRight.green());
        telemetry.addData("Blue Right", U2.colorSensorRight.blue());
        telemetry.addData("Hue Right", hsvValuesRight[0]);

        telemetry.addData("Distance (cm) Right",
                String.format(Locale.US, "%.02f", U2.distanceSensorLeft.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha Right", U2.colorSensorLeft.alpha());
        telemetry.addData("Red Right", U2.colorSensorLeft.red());
        telemetry.addData("Green Right", U2.colorSensorLeft.green());
        telemetry.addData("Blue Right", U2.colorSensorLeft.blue());
        telemetry.addData("Hue Right", hsvValuesRight[0]);



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


        //variable for coordinates
        double coords[] = {777, 777};

        //loop over I.skystone coordiates a few times
        int i = 0;
        while (i < 30) {
            coords = I.skystone_cooridinates();
            i = i + 1;
            sleep(80);
        }

        //get the second coordinate
        //coords[1];
        // if the second coordinate is less than 0, position 1
        //if it's greateer than 0, position 2
        //if it's not found (777), position 3

        if (coords[1] < 0) {
            telemetry.addLine("first skystone seen");
            telemetry.update();

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
            robot.moveInches(67, 0.4, 10);
            U.moveArmEncoder(U.ARM_OUT_POSITION);
            robot.moveInches(-16, 0.25, 10);

        } else if (coords[1] != 777) {
            telemetry.addLine("second skystone seen");
            telemetry.update();

            robot.strafeSeconds(650,-.25);
            robot.moveInches(19, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(1400);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(1000);

            robot.moveInches(-27, -.25, 10);
            //robot.strafeSeconds(2300, -0.25);
            //robot.resetDegrees(0.15);

            robot.rotateDegrees(-85, 0.25);
            robot.strafeSeconds(1000 ,-0.25);

            //goes forward beyond skybridge
            robot.moveInches(60, 0.4, 10);
            U.moveArmEncoder(U.ARM_OUT_POSITION);

            robot.moveInches(-16, 0.3, 10);


        } else {
            telemetry.addLine("third skystone seen");
            telemetry.update();

            robot.strafeSeconds(500, -0.25);
            robot.moveInches(19, 0.28, 10);

            U.moveLiftEncoder(U.LIFT_DOWN_POSITION);
            sleep(1400);

            U.moveArmEncoder(U.ARM_AUTO_PINCH);
            sleep(1000);

            robot.moveInches(-27, -.25, 10);
            //robot.strafeSeconds(2800, -0.25);
            //robot.resetDegrees(0.25);

            robot.rotateDegrees(-85, 0.25);
            robot.strafeSeconds(1000 ,-0.25);

            //goes forward beyond skybridge
            robot.moveInches(75, 0.4, 10);
            U.moveArmEncoder(U.ARM_OUT_POSITION);

            robot.moveInches(-18, 0.3, 10);


        }

        stop();

    }



}
