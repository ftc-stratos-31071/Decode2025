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
    private final boolean holdEnd;
    private final double maxPower;
    private boolean initialized = false;

    // Constructor with just PathChain
    public FollowPath(Follower follower, PathChain path) {
        this(follower, path, true, 1.0);
    }

    // Constructor with PathChain and holdEnd
    public FollowPath(Follower follower, PathChain path, boolean holdEnd) {
        this(follower, path, holdEnd, 1.0);
    }

    // Full constructor with PathChain, holdEnd and maxPower
    public FollowPath(Follower follower, PathChain path, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    // Constructor with just Path
    public FollowPath(Follower follower, Path path) {
        this(follower, path, true, 1.0);
    }

    // Constructor with Path and holdEnd
    public FollowPath(Follower follower, Path path, boolean holdEnd) {
        this(follower, path, holdEnd, 1.0);
    }

    // Full constructor with Path, holdEnd and maxPower
    public FollowPath(Follower follower, Path path, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    @Override
    public boolean isDone() {
        // Initialize the path following on first call
        if (!initialized) {
            if (path instanceof PathChain) {
                // Pedro API: followPath(PathChain path, double maxPower, boolean holdEnd)
                follower.followPath((PathChain) path, maxPower, holdEnd);
            } else if (path instanceof Path) {
                // For single Path, use basic followPath
                follower.followPath((Path) path);
            }
            initialized = true;
        }

        // Update follower and check if done
        follower.update();
        return !follower.isBusy();
    }
}
