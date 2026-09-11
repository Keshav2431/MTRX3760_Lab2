//-----------------------------------------------------------------------------
// CDriveTrain.cpp
//-----------------------------------------------------------------------------

#include "CDriveTrain.h"
#include "CNoiseSource.h"

#include <cmath>

//-----------------------------------------------------------------------------
CDriveTrain::CDriveTrain( float aAxleWidth, CNoiseSource& arNoise )
    :
        mAxleWidth( aAxleWidth ),
        mLeftSpeed( 0.0f ),
        mRightSpeed( 0.0f ),
        mrNoise( arNoise )
{
}


//-----------------------------------------------------------------------------
void CDriveTrain::SetWheelSpeeds( float aLeftSpeed, float aRightSpeed )
{
    mLeftSpeed = aLeftSpeed;
    mRightSpeed = aRightSpeed;
}


//-----------------------------------------------------------------------------
CPose CDriveTrain::Advance( const CPose& arPose, float aTimeStep )
{
    // Working in distance travelled per wheel, rather than speed, lets the
    // random slip be added to each wheel separately before they're combined.
    float LeftDistance = mrNoise.AddWheelSlip( mLeftSpeed * aTimeStep );
    float RightDistance = mrNoise.AddWheelSlip( mRightSpeed * aTimeStep );

    float ForwardDistance = 0.5f * ( LeftDistance + RightDistance );
    float HeadingChange = ( LeftDistance - RightDistance ) / mAxleWidth;

    CPose NewPose = arPose;

    NewPose.mPosition.x += ForwardDistance * std::cos( arPose.mHeading );
    NewPose.mPosition.y += ForwardDistance * std::sin( arPose.mHeading );
    NewPose.mHeading    += HeadingChange;

    return NewPose;
}
