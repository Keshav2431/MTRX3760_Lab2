//-----------------------------------------------------------------------------
// CRender.cpp
//
// Implementation of the raylib wrapper. The :: prefix on the raylib calls says
// "the global one", distinguishing raylib's DrawCircle from our member function
// of the same name.
//-----------------------------------------------------------------------------

#include "CRender.h"

//-----------------------------------------------------------------------------
CRender::CRender()
    :
        mScreenWidth( 800 ),
        mScreenHeight( 600 )
{
    InitWindow( mScreenWidth, mScreenHeight, "MTRX3760 Lab 2 - A5 Noise Bonus" );
    SetTargetFPS( 60 );
}

int CRender::GetScreenWidth() const
{
    return mScreenWidth;
}

int CRender::GetScreenHeight() const
{
    return mScreenHeight;
}

bool CRender::WindowShouldClose()
{
    bool Result = ::WindowShouldClose();
    return Result;
}

void CRender::CloseWindow()
{
    ::CloseWindow();
}

void CRender::BeginDrawing()
{
    ::BeginDrawing();
    ::ClearBackground( BLACK );
}

void CRender::EndDrawing()
{
    ::EndDrawing();
}

void CRender::DrawCircle( Vec2D aPosition, int aRadius, Color aColor )
{
    ::DrawCircle( aPosition.x, aPosition.y, aRadius, aColor );
}

void CRender::DrawLine( Vec2D aStart, Vec2D aEnd, float aThickness, Color aColor )
{
    Vector2 Start = { aStart.x, aStart.y };
    Vector2 End = { aEnd.x, aEnd.y };

    ::DrawLineEx( Start, End, aThickness, aColor );
}

Color CRender::Lighten( Color aColour, float aFraction )
{
    Color Result = aColour;

    Result.r = MixTowardsWhite( aColour.r, aFraction );
    Result.g = MixTowardsWhite( aColour.g, aFraction );
    Result.b = MixTowardsWhite( aColour.b, aFraction );

    return Result;
}

unsigned char CRender::MixTowardsWhite( unsigned char aChannel, float aFraction )
{
    const float White = 255.0f;
    const float Rounding = 0.5f;

    float Value = static_cast<float>( aChannel );
    float Mixed = Value + ( White - Value ) * aFraction;

    return static_cast<unsigned char>( Mixed + Rounding );
}
