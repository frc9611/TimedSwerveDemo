package frc.robot.swerve;

public class SwervePosition {

    public static SwervePosition FRONT_RIGHT = new SwervePosition(SwervePositionSide.RIGHT, SwervePositionPos.FRONT);
    public static SwervePosition FRONT_LEFT = new SwervePosition(SwervePositionSide.LEFT, SwervePositionPos.FRONT);
    public static SwervePosition BACK_RIGHT = new SwervePosition(SwervePositionSide.RIGHT, SwervePositionPos.BACK);
    public static SwervePosition BACK_LEFT = new SwervePosition(SwervePositionSide.LEFT, SwervePositionPos.BACK);

    public enum SwervePositionSide {
        LEFT, RIGHT;
    }

    public enum SwervePositionPos {
        FRONT, BACK;
    }

    private SwervePositionSide side;
    private SwervePositionPos pos;

    public SwervePosition(SwervePositionSide side, SwervePositionPos pos) {
        this.side = side;
        this.pos = pos;
    }

    public SwervePositionSide getSide() {
        return side;
    }

    public SwervePositionPos getPos() {
        return pos;
    }
}
