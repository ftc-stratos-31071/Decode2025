package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import dev.nextftc.core.commands.Command;

/**
 * Command to follow a Pedro PathChain
 */
public class PedroPathingCommand extends Command {
    private final Follower follower;
    private final PathChain path;

    public PedroPathingCommand(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
    }

    public void init() {
        follower.followPath(path);
    }

    public void execute() {
        follower.update();
    }

    @Override
    public boolean isDone() {
        return !follower.isBusy();
    }
}
