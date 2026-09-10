//-----------------------------------------------------------------------------
// CDriveTrain.cpp
//-----------------------------------------------------------------------------

#include "CDriveTrain.h"

#include <cmath>

//-----------------------------------------------------------------------------
CDriveTrain::CDriveTrain( float aAxleWidth )
    :
        mAxleWidth( aAxleWidth ),
        mLeftSpeed( 0.0f ),
        mRightSpeed( 0.0f )
{
}


//-----------------------------------------------------------------------------
void CDriveTrain::SetWheelSpeeds( float aLeftSpeed, float aRightSpeed )
{
    mLeftSpeed = aLeftSpeed;
    mRightSpeed = aRightSpeed;
}


//-----------------------------------------------------------------------------
CPose CDriveTrain::Advance( const CPose& arPose, float aTimeStep ) const
{
    float ForwardSpeed = 0.5f * ( mLeftSpeed + mRightSpeed );
    float AngularSpeed = ( mLeftSpeed - mRightSpeed ) / mAxleWidth;

    CPose NewPose = arPose;

    NewPose.mPosition.x += ForwardSpeed * std::cos( arPose.mHeading ) * aTimeStep;
    NewPose.mPosition.y += ForwardSpeed * std::sin( arPose.mHeading ) * aTimeStep;
    NewPose.mHeading    += AngularSpeed * aTimeStep;

    return NewPose;
}
