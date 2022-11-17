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

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop 3", group="Zhou")
//@Disabled
public class MecanumTeleop3 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum3 robot           = new HardwareMecanum3();   // Use the updated Mecanum's hardware
    double howfast;
    double shift;
    double turn;
    double          armOffset      = 0;                       // initial arm position
    final double    ARM_SPEED      = 0.001 ;                   // sets rate to move servo
    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            howfast = gamepad1.left_stick_y;
            shift = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            robot.leftFront.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            robot.rightFront.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
            robot.leftBack.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            robot.rightBack.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);

            if (gamepad1.left_stick_x > 0)
                robot.goRight(shift);
            else if (gamepad1.left_stick_x < 0)
                robot.goRight(shift);
            else if (gamepad1.left_stick_x == 0)
                robot.goRight(0);

            //use trigger to lift
            if (gamepad1.left_trigger > 0)
                robot.lift.setPower(gamepad1.left_trigger); // lift up
            else if (gamepad1.right_trigger > 0)
                robot.lift.setPower(-1 * gamepad1.right_trigger); // lift down
            else
                robot.lift.setPower(0);

            // use bumpers to spin
            if (gamepad1.left_bumper) {
                robot.spinnerL.setPower(-1);  // spin duck inward
                robot.spinnerR.setPower(-1); // spin duck inward
            } else if (gamepad1.right_bumper) {
                robot.spinnerL.setPower(1); // spin duck outward
                robot.spinnerR.setPower(1); // spin duck outward
            } else {
                robot.spinnerL.setPower(0);
                robot.spinnerR.setPower(0);
            }

                //use X&B to roll
                if (gamepad1.x)
                    robot.roller.setPower(1);  // roll out
                else if (gamepad1.b)
                    robot.roller.setPower(-1);  // roll in
                else
                    robot.roller.setPower(0);

                //use Y&A to move arm
                if (gamepad1.y)
                    armOffset += ARM_SPEED;
                else if (gamepad1.a)
                    armOffset -= ARM_SPEED;

                armOffset = Range.clip(armOffset, -0.45, 0.65);
                robot.arm.setPosition(0.0+armOffset);

                /*

                Use dpad_up and dpad_down to extend and retract cheater
                if (gamepad1.dpad_up)
                    robot.cheater.setPower(1); // extend cheater
                else if (gamepad1.dpad_down)
                    robot.cheater.setPower(-1); //retract cheater
                else
                    robot.cheater.setPower(0);*/


                // Send telemetry message to signify robot running;
                // telemetry.addData("claw",  "Offset = %.2f", clawOffset);
                // telemetry.addData("left",  "%.2f", left);
                //telemetry.addData("right", "%.2f", right);
                //telemetry.update();
            }

        }
    }
