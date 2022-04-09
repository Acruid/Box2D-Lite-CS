/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

using System.Diagnostics;

namespace Engine.Box2D;

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

enum Axis
{
    FACE_A_X,
    FACE_A_Y,
    FACE_B_X,
    FACE_B_Y
};

enum EdgeNumbers
{
    NO_EDGE = 0,
    EDGE1,
    EDGE2,
    EDGE3,
    EDGE4
};

struct ClipVertex
{
    public Vec2 v;
    public FeaturePair fp;
};

internal static class Collision
{
    static void Flip(ref FeaturePair fp)
    {
	    FMath.Swap(ref fp.e.inEdge1, ref fp.e.inEdge2);
        FMath.Swap(ref fp.e.outEdge1, ref fp.e.outEdge2);
    }

    static int ClipSegmentToLine(Span<ClipVertex> vOut, Span<ClipVertex> vIn,
					      in Vec2 normal, float offset, byte clipEdge)
    {
	    // Start with no output points
	    int numOut = 0;

	    // Calculate the distance of end points to the line
	    float distance0 = Vec2.Dot(normal, vIn[0].v) - offset;
	    float distance1 = Vec2.Dot(normal, vIn[1].v) - offset;

	    // If the points are behind the plane
	    if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	    if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	    // If the points are on different sides of the plane
	    if (distance0 * distance1 < 0.0f)
	    {
		    // Find intersection point of edge and plane
		    float interp = distance0 / (distance0 - distance1);
		    vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		    if (distance0 > 0.0f)
		    {
			    vOut[numOut].fp = vIn[0].fp;
			    vOut[numOut].fp.e.inEdge1 = clipEdge;
			    vOut[numOut].fp.e.inEdge2 = (byte)EdgeNumbers.NO_EDGE;
		    }
		    else
		    {
			    vOut[numOut].fp = vIn[1].fp;
			    vOut[numOut].fp.e.outEdge1 = clipEdge;
			    vOut[numOut].fp.e.outEdge2 = (byte)EdgeNumbers.NO_EDGE;
		    }
		    ++numOut;
	    }

	    return numOut;
    }

    static void ComputeIncidentEdge(Span<ClipVertex> c, in Vec2 h, in Vec2 pos,
								    in Mat22 Rot, in Vec2 normal)
    {
		Debug.Assert(c.Length == 2);

	    // The normal is from the reference box. Convert it
	    // to the incident boxe's frame and flip sign.
	    Mat22 RotT = Rot.Transpose();
	    Vec2 n = -(RotT * normal);
	    Vec2 nAbs = Vec2.Abs(n);

	    if (nAbs.x > nAbs.y)
	    {
		    if (MathF.Sign(n.x) > 0.0f)
		    {
			    c[0].v.Set(h.x, -h.y);
			    c[0].fp.e.inEdge2 = (byte)EdgeNumbers.EDGE3;
			    c[0].fp.e.outEdge2 = (byte)EdgeNumbers.EDGE4;

			    c[1].v.Set(h.x, h.y);
			    c[1].fp.e.inEdge2 = (byte)EdgeNumbers.EDGE4;
			    c[1].fp.e.outEdge2 = (byte)EdgeNumbers.EDGE1;
		    }
		    else
		    {
			    c[0].v.Set(-h.x, h.y);
			    c[0].fp.e.inEdge2 = (byte)EdgeNumbers.EDGE1;
			    c[0].fp.e.outEdge2 = (byte)EdgeNumbers.EDGE2;

			    c[1].v.Set(-h.x, -h.y);
			    c[1].fp.e.inEdge2 = (byte)EdgeNumbers.EDGE2;
			    c[1].fp.e.outEdge2 = (byte)EdgeNumbers.EDGE3;
		    }
	    }
	    else
	    {
		    if (MathF.Sign(n.y) > 0.0f)
		    {
			    c[0].v.Set(h.x, h.y);
			    c[0].fp.e.inEdge2 = (byte)EdgeNumbers.EDGE4;
			    c[0].fp.e.outEdge2 = (byte)EdgeNumbers.EDGE1;

			    c[1].v.Set(-h.x, h.y);
			    c[1].fp.e.inEdge2 = (byte)EdgeNumbers.EDGE1;
			    c[1].fp.e.outEdge2 = (byte)EdgeNumbers.EDGE2;
		    }
		    else
		    {
			    c[0].v.Set(-h.x, -h.y);
			    c[0].fp.e.inEdge2 = (byte)EdgeNumbers.EDGE2;
			    c[0].fp.e.outEdge2 = (byte)EdgeNumbers.EDGE3;

			    c[1].v.Set(h.x, -h.y);
			    c[1].fp.e.inEdge2 = (byte)EdgeNumbers.EDGE3;
			    c[1].fp.e.outEdge2 = (byte)EdgeNumbers.EDGE4;
		    }
	    }

	    c[0].v = pos + Rot * c[0].v;
	    c[1].v = pos + Rot * c[1].v;
    }

    // The normal points from A to B
    public static int Collide(Contact[] contacts, ref Body bodyA, ref Body bodyB)
    {
	    // Setup
	    Vec2 hA = 0.5f * bodyA.width;
	    Vec2 hB = 0.5f * bodyB.width;

	    Vec2 posA = bodyA.position;
	    Vec2 posB = bodyB.position;

	    Mat22 RotA = new(bodyA.rotation);
        Mat22 RotB = new(bodyB.rotation);

	    Mat22 RotAT = RotA.Transpose();
	    Mat22 RotBT = RotB.Transpose();

	    Vec2 a1 = RotA.col1, a2 = RotA.col2;
	    Vec2 b1 = RotB.col1, b2 = RotB.col2;

	    Vec2 dp = posB - posA;
	    Vec2 dA = RotAT * dp;
	    Vec2 dB = RotBT * dp;

	    Mat22 C = RotAT * RotB;
	    Mat22 absC = Mat22.Abs(C);
	    Mat22 absCT = absC.Transpose();

	    // Box A faces
	    Vec2 faceA = Vec2.Abs(dA) - hA - absC * hB;
	    if (faceA.x > 0.0f || faceA.y > 0.0f)
		    return 0;

	    // Box B faces
	    Vec2 faceB = Vec2.Abs(dB) - absCT * hA - hB;
	    if (faceB.x > 0.0f || faceB.y > 0.0f)
		    return 0;

	    // Find best axis
	    Axis axis;
	    float separation;
	    Vec2 normal;

	    // Box A faces
	    axis = Axis.FACE_A_X;
	    separation = faceA.x;
	    normal = dA.x > 0.0f ? RotA.col1 : -RotA.col1;

	    const float relativeTol = 0.95f;
	    const float absoluteTol = 0.01f;

	    if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
	    {
		    axis = Axis.FACE_A_Y;
		    separation = faceA.y;
		    normal = dA.y > 0.0f ? RotA.col2 : -RotA.col2;
	    }

	    // Box B faces
	    if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
	    {
		    axis = Axis.FACE_B_X;
		    separation = faceB.x;
		    normal = dB.x > 0.0f ? RotB.col1 : -RotB.col1;
	    }

	    if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
	    {
		    axis = Axis.FACE_B_Y;
		    separation = faceB.y;
		    normal = dB.y > 0.0f ? RotB.col2 : -RotB.col2;
	    }

	    // Setup clipping plane data based on the separating axis
	    Vec2 frontNormal, sideNormal;
	    Span<ClipVertex> incidentEdge = stackalloc ClipVertex[2];
	    float front, negSide, posSide;
	    byte negEdge, posEdge;

		// Initialize variables
        {
            frontNormal = default;
            front = default;
            sideNormal = default;
            negSide = default;
            posSide =  default;
            negEdge = default;
            posEdge = default;
        }

	    // Compute the clipping lines and the line segment to be clipped.
	    switch (axis)
	    {
	    case Axis.FACE_A_X:
		    {
			    frontNormal = normal;
			    front = Vec2.Dot(posA, frontNormal) + hA.x;
			    sideNormal = RotA.col2;
			    float side = Vec2.Dot(posA, sideNormal);
			    negSide = -side + hA.y;
			    posSide =  side + hA.y;
			    negEdge = (byte)EdgeNumbers.EDGE3;
			    posEdge = (byte)EdgeNumbers.EDGE1;
			    ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
		    }
		    break;

	    case Axis.FACE_A_Y:
		    {
			    frontNormal = normal;
			    front = Vec2.Dot(posA, frontNormal) + hA.y;
			    sideNormal = RotA.col1;
			    float side = Vec2.Dot(posA, sideNormal);
			    negSide = -side + hA.x;
			    posSide =  side + hA.x;
			    negEdge = (byte)EdgeNumbers.EDGE2;
			    posEdge = (byte)EdgeNumbers.EDGE4;
			    ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
		    }
		    break;

	    case Axis.FACE_B_X:
		    {
			    frontNormal = -normal;
			    front = Vec2.Dot(posB, frontNormal) + hB.x;
			    sideNormal = RotB.col2;
			    float side = Vec2.Dot(posB, sideNormal);
			    negSide = -side + hB.y;
			    posSide =  side + hB.y;
			    negEdge = (byte)EdgeNumbers.EDGE3;
			    posEdge = (byte)EdgeNumbers.EDGE1;
			    ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
		    }
		    break;

	    case Axis.FACE_B_Y:
		    {
			    frontNormal = -normal;
			    front = Vec2.Dot(posB, frontNormal) + hB.y;
			    sideNormal = RotB.col1;
			    float side = Vec2.Dot(posB, sideNormal);
			    negSide = -side + hB.x;
			    posSide =  side + hB.x;
			    negEdge = (byte)EdgeNumbers.EDGE2;
			    posEdge = (byte)EdgeNumbers.EDGE4;
			    ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
		    }
		    break;
        }

	    // clip other face with 5 box planes (1 face plane, 4 edge planes)

	    Span<ClipVertex> clipPoints1 = stackalloc ClipVertex[2];
        Span<ClipVertex> clipPoints2 = stackalloc ClipVertex[2];
	    int np;

	    // Clip to box side 1
	    np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

	    if (np < 2)
		    return 0;

	    // Clip to negative box side 1
	    np = ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, posSide, posEdge);

	    if (np < 2)
		    return 0;

	    // Now clipPoints2 contains the clipping points.
	    // Due to roundoff, it is possible that clipping removes all points.

	    int numContacts = 0;
	    for (int i = 0; i < 2; ++i)
	    {
		    separation = Vec2.Dot(frontNormal, clipPoints2[i].v) - front;

		    if (separation <= 0)
		    {
			    contacts[numContacts].separation = separation;
			    contacts[numContacts].normal = normal;
			    // slide contact point onto reference face (easy to cull)
			    contacts[numContacts].position = clipPoints2[i].v - separation * frontNormal;
			    contacts[numContacts].feature = clipPoints2[i].fp;
			    if (axis == Axis.FACE_B_X || axis == Axis.FACE_B_Y)
				    Flip(ref contacts[numContacts].feature);
			    ++numContacts;
		    }
	    }

	    return numContacts;
    }
}