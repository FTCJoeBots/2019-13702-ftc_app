package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

/**
 *Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
 *code and understand this code for the possibility that a question may be asked related to TeleOp and
 *you should be able to explain in good detail everything in this code.
 *11/16/17-> Changed all gamepad's in code to correct gamepad (i.e some gamepad1's to gamepad2)
 ***11/18/17-> Competition Notes below
 *Notes-> Autonomous is incorrect, Not much was wrong from a software sandpoint but hardware issues were fixed
 *Autonomous issues included: Incorrect spinning causing us to move out of destination,
 *To much time on the down motion of the clamp and arm.
 *These issues are still not resolved
 * Recomendation for autonomous issues(Not Offical):Fine tune the timer on the clamp
 * Fine tune the movements and LOWER the TIME OF MOVEMENT in autonomous.
 * List of issues at Comp(1)-> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1r_liipKBU7GHfONdxq9E6d4f7zikcCuXwDL2bsQfwm0/edit?usp=sharing
 *G-Sheet of time VS Heading for autonomous -> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1pqv0iN94fFd5KvX1YIWP7z39HgpURXsscn0zPujs1q4/edit?usp=sharing
*/
@TeleOp(name="Servo TeleOp Test", group="TeleOp")

public class servoTele extends LinearOpMode {

    double leftIntakeServoPos = 0.3;

    HardwareJoeBot2019 robot = new HardwareJoeBot2019();
    Utility13702       U = new Utility13702();

    @Override
    public void runOpMode() throws InterruptedException {

        U.init(hardwareMap, this);
        robot.init(hardwareMap, this);

        telemetry.addLine("initialized");

        waitForStart();

        //start of loop
        while (opModeIsActive()) {

            if(gamepad1.dpad_up == true){
                leftIntakeServoPos += 0.05;
                U.leftIntakeServo.setPosition(leftIntakeServoPos);
                gamepad1.dpad_up=false;
                sleep(1000);

            }

            if(gamepad1.dpad_down == true){
                leftIntakeServoPos -= 0.05;
                U.leftIntakeServo.setPosition(leftIntakeServoPos);
                sleep(1000);
            }



            //------------------------------------------
            //-------------------------------------------



            // Update Telemetry
            telemetry.addData(">", "Press Stop to end test.");

            if (gamepad1.a) {
                telemetry.addLine("Button A is pressed on pad 1");
            } else if (gamepad1.b) {
                telemetry.addLine("Button B is pressed on pad 1");
            } else {
                telemetry.addLine("Neither button is pressed on pad 1");
            }

            if (gamepad2.a) {
                telemetry.addLine("Button A is pressed on pad 2");
            } else if (gamepad2.b) {
                telemetry.addLine("Button B is pressed on pad 2");
            } else {
                telemetry.addLine("Neither button is pressed on pad 2");
            }

            telemetry.addData("leftIntakeServo", leftIntakeServoPos);
            telemetry.update();

            telemetry.update();
            idle();


        }//end while
    }
}