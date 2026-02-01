package com.example.meepmeeptesting1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeeptesting1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(59, 12, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(31,30), Math.toRadians(-273))
                .waitSeconds(.1)
                .strafeToConstantHeading(
                        new Vector2d(29, 72)
                )
                .strafeToLinearHeading(new Vector2d(59, 12), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(7.5,35), Math.toRadians(-273))
                .waitSeconds(.1)
                .strafeToConstantHeading(
                        new Vector2d(5.5, 72)
                )
                .strafeToLinearHeading(new Vector2d(59, 12), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(35, 20), Math.toRadians(-180))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}