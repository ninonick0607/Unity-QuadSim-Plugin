using SimCore.Common;

namespace DroneCore.Interfaces
{
    public interface ICommandSource
    {
        Axis4 GetCommandValue();
        GoalMode GetGoalMode();
        InputSource GetActiveSource();
    }
}