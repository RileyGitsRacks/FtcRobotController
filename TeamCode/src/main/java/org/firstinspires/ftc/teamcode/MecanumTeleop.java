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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@TeleOp(name="Mecanum: Teleop", group="Mecanum")
// @Disabled
public class MecanumTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot           = new HardwareMecanum();   // Use a Mecanum's hardware

    @Override
    public void runOpMode() {
        double x1 = 0; // set the initial value, that will change when the joystick is moved
        double y1 = 0;

        double fortyFiveInRads = -Math.PI/4;
        double cosine45 = Math.cos(fortyFiveInRads);
        double sine45 = Math.sin(fortyFiveInRads);

        double x2 = 0;
        double y2 = 0;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double spin = gamepad1.right_stick_x;

            if (Math.abs(spin) > 0.1) {
                // if someone is moving the right joystick, spin
                robot.frontRightDrive.setPower(-spin);
                robot.backRightDrive.setPower(-spin);

                robot.frontLeftDrive.setPower(spin);
                robot.backLeftDrive.setPower(spin);
            }
        else {
                // if no one is pressing the right joystick, do the normal driving code
                y1 = -gamepad1.left_stick_y;
                x1 = gamepad1.left_stick_x;


                // need to rotate 45 degrees

                y2 = y1 * cosine45 + x1 * sine45;
                x2 = x1 * cosine45 - y1 * sine45;


                // Output the safe vales to the motor drives.
                robot.frontLeftDrive.setPower(x2);
                robot.backRightDrive.setPower(x2);

                robot.frontRightDrive.setPower(y2);
                robot.backLeftDrive.setPower(y2);
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("y1",  "Offset = %.2f", y1);
            telemetry.addData("x1",  "%.2f", x1);
            telemetry.addData("y2",  "%.2f", y2);
            telemetry.addData("x2",  "%.2f", x2);
            telemetry.update();

            // Pace this loop
            sleep(50);
        }
    }
}
