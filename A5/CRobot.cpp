//-----------------------------------------------------------------------------
// CRobot.cpp
//
// The shared robot mechanics: moving one step, dealing with wall collisions
// (undo the move but keep the new heading, and only count the hit once),
// checking whether the lap is finished, remembering the trail, and drawing it
// all. Sensing and steering are the only parts left for a specific robot to
// write.
//-----------------------------------------------------------------------------

#include "CRobot.h"
#include "CRoom.h"

#include <cmath>
#include <cstddef>

//---A little over two steps at normal speed: close enough that the trail
//   still looks like a smooth curve, but far enough apart that a robot
//   turning almost on the spot doesn't pile up points where it barely moved.
const float CRobot::kTrailPointSpacing = 4.0f;

//---A robot has to get properly clear of its start point before coming back
//   counts as finishing a lap - otherwise it could be credited a lap within
//   its first few steps. Both loops in this simulation are much wider than
//   150 units, so a robot that really does a lap always clears this.-------
const float CRobot::kLapLeaveDistance = 150.0f;
const float CRobot::kLapReturnDistance = 30.0f;


//-----------------------------------------------------------------------------
// Sets the robot's fixed measurements, places it at its start pose, and
// remembers that starting point both as the first point of its trail and as
// the spot the lap test measures distance from.
//-----------------------------------------------------------------------------
CRobot::CRobot( const CPose& arStartPose, const CRoom& arRoom,
                CNoiseSource& arNoise, Color aBodyColour )
    :
        mRadius( 15.0f ),
        mAxleWidth( 26.0f ),
        mBaseSpeed( 40.0f ),
        mTimeStep( 0.05f ),
        mPose( arStartPose ),
        mDriveTrain( mAxleWidth, arNoise ),
        mBodyColour( aBodyColour ),
        mStartPosition( arStartPose.mPosition ),
        mHasLeftStart( false ),
        mLapDone( false ),
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
// speeds, try to move, deal with a wall if it hits one, and check whether
// that move just finished the lap.
void CRobot::Update()
{
    // Once the lap is finished the robot just stops: its trail stays on
    // screen, but it doesn't drive any further or draw over its own picture.
    if( !mLapDone )
    {
        Sense();
        SteerFromSensors();

        CPose TentativePose = mDriveTrain.Advance( mPose, mTimeStep );

        if( HasCollided( TentativePose ) )
        {
            // Undo the move, but keep the new heading, so steering still has
            // a chance to turn the robot away from the wall next step.
            mPose.mHeading = TentativePose.mHeading;

            if( !mWasColliding )
            {
                ++mCollisionCount;
            }
            mWasColliding = true;
        }
        else
        {
            mPose = TentativePose;
            mWasColliding = false;
        }

        CheckLap();
        ExtendTrail();
        ++mUpdateCount;
    }
}


//-----------------------------------------------------------------------------
void CRobot::DrawTrail( CRender& arRender ) const
{
    const float TrailThickness = 1.5f;

    // A line joining each stored point to the next, in the robot's own colour.
    for( std::size_t i = 1; i < mTrail.size(); ++i )
    {
        arRender.DrawLine( mTrail[i - 1], mTrail[i], TrailThickness, mBodyColour );
    }
}


//-----------------------------------------------------------------------------
void CRobot::DrawBody( CRender& arRender ) const
{
    const float HeadingLineThickness = 2.0f;

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
bool CRobot::HasFinishedLap() const
{
    return mLapDone;
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


//-----------------------------------------------------------------------------
// Works out how far the robot currently is from where it started. The first
// check remembers, once and for all, that the robot has properly driven away
// from home - that's mHasLeftStart, and it only ever gets set to true, never
// back to false. The second check then looks for the robot coming back close
// to home again, but only counts it once mHasLeftStart is already true. That
// two-step check is what stops a robot being credited with a lap in its very
// first few steps, before it has gone anywhere.
void CRobot::CheckLap()
{
    float OffsetX = mPose.mPosition.x - mStartPosition.x;
    float OffsetY = mPose.mPosition.y - mStartPosition.y;
    float DistanceFromStart = std::sqrt( OffsetX * OffsetX + OffsetY * OffsetY );

    if( DistanceFromStart > kLapLeaveDistance )
    {
        mHasLeftStart = true;
    }

    if( mHasLeftStart && ( DistanceFromStart < kLapReturnDistance ) )
    {
        mLapDone = true;
    }
}


//-----------------------------------------------------------------------------
void CRobot::ExtendTrail()
{
    // Copied by value: push_back below can move the trail's storage around,
    // which would leave a reference to the old storage pointing at nothing.
    Vec2D LastPoint = mTrail.back();

    float DeltaX = mPose.mPosition.x - LastPoint.x;
    float DeltaY = mPose.mPosition.y - LastPoint.y;

    // Compared as squared distances, so there's no need for a square root
    // check on every single step.
    if( DeltaX * DeltaX + DeltaY * DeltaY >= kTrailPointSpacing * kTrailPointSpacing )
    {
        mTrail.push_back( mPose.mPosition );
    }
}
