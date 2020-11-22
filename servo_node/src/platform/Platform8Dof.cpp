#include "platform/Platform8Dof.h"

platform::Platform8Dof::Platform8Dof(platform::Platform8DofParameters parameters)
    : parameters(parameters)
{
}

bool platform::Platform8Dof::setSpeed(double speedRotateClockwise, double speedMoveForward)
{
    // TODO: update worker thread controlling platform legs 
    return false;
}
