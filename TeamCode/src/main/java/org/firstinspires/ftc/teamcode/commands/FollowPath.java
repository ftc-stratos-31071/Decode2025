package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import dev.nextftc.core.commands.Command;

/**
 * Command to follow a Pedro Path or PathChain.
 * This is the NextFTC-style implementation of FollowPath.
 */
public class FollowPath extends Command {
    private final Follower follower;
    private final Object path; // Can be Path or PathChain

    // Constructor with just PathChain
    public FollowPath(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
    }

    // Constructor with just Path
    public FollowPath(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
    }

    @Override
    public void start() {
        // Initialize the path following when command starts
        if (path instanceof PathChain) {
            follower.followPath((PathChain) path);
        } else if (path instanceof Path) {
            follower.followPath((Path) path);
        }
    }

    @Override
    public void update() {
        // Update follower every loop - critical for Pedro to work!
        follower.update();
    }

    @Override
    public boolean isDone() {
        // Command is done when follower is no longer busy
        return !follower.isBusy();
    }
}
