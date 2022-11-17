/* Copyright (c) 2018 FIRST. All rights reserved.
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

@Autonomous(name = "BlueSpin3", group = "Zhou")
@Disabled
public class BlueSpin3 extends LinearOpMode {

    HardwareMecanum3      robot   = new HardwareMecanum3();   // Use the 3rd hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     H_SPEED = 0.8;
    static final double     M_SPEED = 0.5;
    static final double     S_SPEED = 0.3;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            robot.reset();
            robot.rightInches(S_SPEED,5,5); //align the left spinner
            robot.forwardInches(S_SPEED,-16,5); //approach carousel
            robot.forwardInches(S_SPEED, -1,1); //ensure contact
            robot.spinnerL.setPower(1); //spin
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()<5) {
                telemetry.addData("spinning", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            robot.spinnerL.setPower(0); //stop spinner
            robot.rightInches(M_SPEED,20,5);    //go to storage
            robot.forwardInches(S_SPEED,-10,3);  //back to wall
            robot.roller.setPower(1);   //spit out cube
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()<3) {
                telemetry.addData("rolling", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            robot.roller.setPower(0); // stop roller
            robot.liftUp(S_SPEED, 5, 2); //lift up away from cube

            stop();

        }
    }

}