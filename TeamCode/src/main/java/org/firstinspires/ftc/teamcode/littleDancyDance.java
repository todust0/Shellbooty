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



@Autonomous(name="Little Dancy Dance", group="Robot")
@Disabled
public class littleDancyDance extends LinearOpMode {


    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.3;

    @Override
    public void runOpMode() {


        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackMotor");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Shellbooty", "Big");    //
        telemetry.update();


        waitForStart();



        //Slide to the left
        leftFrontDrive.setPower(BACKWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(BACKWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //wait for a sec
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Slide to the right
        leftFrontDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(BACKWARD_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(BACKWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //wait for a sec
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //cris cross
        leftFrontDrive.setPower(-TURN_SPEED);
        rightFrontDrive.setPower(TURN_SPEED);
        rightBackDrive.setPower(TURN_SPEED);
        leftBackDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFrontDrive.setPower(TURN_SPEED);
        rightFrontDrive.setPower(-TURN_SPEED);
        rightBackDrive.setPower(-TURN_SPEED);
        leftBackDrive.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //cris cross
        leftFrontDrive.setPower(-TURN_SPEED);
        rightFrontDrive.setPower(TURN_SPEED);
        rightBackDrive.setPower(TURN_SPEED);
        leftBackDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFrontDrive.setPower(TURN_SPEED);
        rightFrontDrive.setPower(-TURN_SPEED);
        rightBackDrive.setPower(-TURN_SPEED);
        leftBackDrive.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Wait for a sec
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.75 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //do a little twirl
        leftFrontDrive.setPower(-TURN_SPEED);
        rightFrontDrive.setPower(TURN_SPEED);
        rightBackDrive.setPower(TURN_SPEED);
        leftBackDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
