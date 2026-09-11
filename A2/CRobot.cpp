//-----------------------------------------------------------------------------
// CRobot.cpp
//
// This is the same robot behaviour A1's CWallFollowerRobot used to have on its
// own: moving one step, dealing with wall collisions (undo the move but keep
// the new heading, and only count the hit once), remembering the trail, and
// drawing it all. Sensing and steering are the only parts left for a specific
// robot to write.
//-----------------------------------------------------------------------------

#include "CRobot.h"
#include "CRoom.h"

#include <cmath>
#include <cstddef>
#include <iostream>

//-----------------------------------------------------------------------------
// Sets the robot's fixed measurements, places it at its start pose, and adds
// that starting point as the first point of its trail.
//-----------------------------------------------------------------------------
CRobot::CRobot( const CPose& arStartPose, const CRoom& arRoom, Color aBodyColour )
    :
        mRadius( 15.0f ),
        mAxleWidth( 26.0f ),
        mBaseSpeed( 40.0f ),
        mTimeStep( 0.05f ),
        mPose( arStartPose ),
        mDriveTrain( mAxleWidth ),
        mBodyColour( aBodyColour ),
        mUpdateCount( 0 ),
        mCollisionCount( 0 ),
        mWasColliding( false ),
        mrRoom( arRoom )
{
    mTrail.push_back( mPose.mPosition );
}


//-----------------------------------------------------------------------------
CRobot::~CRobot()
{
}


//-----------------------------------------------------------------------------
// Runs the robot through one time step: check its sensors, decide on wheel
// speeds, try to move, and deal with a wall if it hits one.
void CRobot::Update()
{
    Sense();
    SteerFromSensors();

    CPose TentativePose = mDriveTrain.Advance( mPose, mTimeStep );

    if( HasCollided( TentativePose ) )
    {
        // Undo the move, but keep the new heading, so steering still has a
        // chance to turn the robot away from the wall on the next step.
        mPose.mHeading = TentativePose.mHeading;

        if( !mWasColliding )
        {
            ++mCollisionCount;
            std::cout << Name() << " collision #" << mCollisionCount
                      << " at update " << mUpdateCount << std::endl;
        }
        mWasColliding = true;
    }
    else
    {
        mPose = TentativePose;
        mWasColliding = false;
    }

    mTrail.push_back( mPose.mPosition );
    ++mUpdateCount;
}


//-----------------------------------------------------------------------------
void CRobot::Draw( CRender& arRender ) const
{
    const float TrailThickness = 1.5f;
    const float HeadingLineThickness = 2.0f;

    // The trail: a line joining each stored point to the next, in the
    // robot's own colour so the two robots' trails can be told apart.
    for( std::size_t i = 1; i < mTrail.size(); ++i )
    {
        arRender.DrawLine( mTrail[i - 1], mTrail[i], TrailThickness, mBodyColour );
    }

    // The body.
    arRender.DrawCircle( mPose.mPosition, static_cast<int>( mRadius ), mBodyColour );

    // A short line from the centre out to the edge, pointing the way the
    // robot is facing.
    Vec2D HeadingEnd
    {
        mPose.mPosition.x + mRadius * std::cos( mPose.mHeading ),
        mPose.mPosition.y + mRadius * std::sin( mPose.mHeading )
    };
    arRender.DrawLine( mPose.mPosition, HeadingEnd, HeadingLineThickness, BLACK );
}


//-----------------------------------------------------------------------------
int CRobot::GetUpdateCount() const
{
    return mUpdateCount;
}


//-----------------------------------------------------------------------------
int CRobot::GetCollisionCount() const
{
    return mCollisionCount;
}


//-----------------------------------------------------------------------------
const CPose& CRobot::Pose() const
{
    return mPose;
}


//-----------------------------------------------------------------------------
const CRoom& CRobot::Room() const
{
    return mrRoom;
}


//-----------------------------------------------------------------------------
void CRobot::SetWheelSpeeds( float aLeft, float aRight )
{
    mDriveTrain.SetWheelSpeeds( aLeft, aRight );
}


//-----------------------------------------------------------------------------
float CRobot::BaseSpeed() const
{
    return mBaseSpeed;
}


//-----------------------------------------------------------------------------
bool CRobot::HasCollided( const CPose& arTentativePose ) const
{
    return mrRoom.IsColliding( arTentativePose.mPosition, mRadius );
}
