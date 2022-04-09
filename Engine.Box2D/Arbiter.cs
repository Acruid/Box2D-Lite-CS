/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

using System.Runtime.InteropServices;
using OpenTK.Graphics.OpenGL;

namespace Engine.Box2D;

struct Edges
{
    public byte inEdge1;
    public byte outEdge1;
    public byte inEdge2;
    public byte outEdge2;
}

[StructLayout(LayoutKind.Explicit)]
struct FeaturePair
{
    [FieldOffset(0)]
    public Edges e;

    [FieldOffset(0)]
	public int value;
};

struct Contact
{
    public Vec2 position;
    public Vec2 normal;
    public Vec2 r1, r2;
    public float separation;
    public float Pn;	// accumulated normal impulse
    public float Pt;	// accumulated tangent impulse
    public float Pnb;	// accumulated normal impulse for position bias
    public float massNormal;
    public float massTangent;
    public float bias;
    public FeaturePair feature;
};

readonly struct ArbiterKey : IEquatable<ArbiterKey>
{
	public ArbiterKey(BodyIndex b1, BodyIndex b2)
	{
		if (b1 < b2)
		{
			body1 = b1; body2 = b2;
		}
		else
		{
			body1 = b2; body2 = b1;
		}
	}

    readonly BodyIndex body1;
    readonly BodyIndex body2;

    // This is used by std::set
    public static bool operator < (in ArbiterKey a1, in ArbiterKey a2)
    {
        if (a1.body1 < a2.body1)
            return true;

        if (a1.body1 == a2.body1 && a1.body2 < a2.body2)
            return true;

        return false;
    }

    public static bool operator >(ArbiterKey a1, ArbiterKey a2)
    {
        throw new NotImplementedException();
    }

    #region Equality

    public bool Equals(ArbiterKey other)
    {
        return body1.Equals(other.body1) && body2.Equals(other.body2);
    }

    public override bool Equals(object? obj)
    {
        return obj is ArbiterKey other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(body1, body2);
    }

    public static bool operator ==(ArbiterKey left, ArbiterKey right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(ArbiterKey left, ArbiterKey right)
    {
        return !left.Equals(right);
    }

    #endregion
};

struct Arbiter
{
	const int MAX_POINTS = 2;

	public Arbiter(Span<Body> bodies, BodyIndex b1, BodyIndex b2)
    {
        if (b1 < b2)
        {
            body1Ref = b1;
            body2Ref = b2;
        }
        else
        {
            body1Ref = b2;
            body2Ref = b1;
        }

        ref Body body1 = ref bodies[b1];
        ref Body body2 = ref bodies[b2];

        numContacts = Collision.Collide(contacts, ref body1, ref body2);

        friction = MathF.Sqrt(body1.friction * body2.friction);

        GL.PointSize(4.0f);
        GL.Color3(1.0f, 0.0f, 0.0f);
        GL.Begin(BeginMode.Points);
        for (int i = 0; i < numContacts; ++i)
        {
            GL.Vertex2(contacts[i].position.x, contacts[i].position.y);
        }
        GL.End();
        GL.PointSize(1.0f);
    }

    public void Update(Contact[] newContacts, int numNewContacts)
    {
        Span<Contact> mergedContacts = stackalloc Contact[2];

        for (int i = 0; i < numNewContacts; ++i)
        {
            ref Contact cNew = ref newContacts[i];
            int k = -1;
            for (int j = 0; j < numContacts; ++j)
            {
                ref Contact cOld = ref contacts[j];
                if (cNew.feature.value == cOld.feature.value)
                {
                    k = j;
                    break;
                }
            }

            if (k > -1)
            {
                ref Contact c = ref mergedContacts[i];
                ref Contact cOld = ref contacts[k];

                // deep copy contact
                mergedContacts[i] = newContacts[i];
                if (World.warmStarting)
                {
                    c.Pn = cOld.Pn;
                    c.Pt = cOld.Pt;
                    c.Pnb = cOld.Pnb;
                }
                else
                {
                    c.Pn = 0.0f;
                    c.Pt = 0.0f;
                    c.Pnb = 0.0f;
                }
            }
            else
            {
                mergedContacts[i] = newContacts[i];
            }
        }

        for (int i = 0; i < numNewContacts; ++i)
            contacts[i] = mergedContacts[i];

        numContacts = numNewContacts;
    }

    public void PreStep(Span<Body> bodies, float inv_dt)
    {
        ref Body body1 = ref bodies[body1Ref];
        ref Body body2 = ref bodies[body2Ref];

        const float k_allowedPenetration = 0.01f;
        float k_biasFactor = World.positionCorrection ? 0.2f : 0.0f;

        for (int i = 0; i < numContacts; ++i)
        {
            ref Contact c = ref contacts[i];

            Vec2 r1 = c.position - body1.position;
            Vec2 r2 = c.position - body2.position;

            // Precompute normal mass, tangent mass, and bias.
            float rn1 = Vec2.Dot(r1, c.normal);
            float rn2 = Vec2.Dot(r2, c.normal);
            float kNormal = body1.invMass + body2.invMass;
            kNormal += body1.invI * (Vec2.Dot(r1, r1) - rn1 * rn1) + body2.invI * (Vec2.Dot(r2, r2) - rn2 * rn2);
            c.massNormal = 1.0f / kNormal;

            Vec2 tangent = Vec2.Cross(c.normal, 1.0f);
            float rt1 = Vec2.Dot(r1, tangent);
            float rt2 = Vec2.Dot(r2, tangent);
            float kTangent = body1.invMass + body2.invMass;
            kTangent += body1.invI * (Vec2.Dot(r1, r1) - rt1 * rt1) + body2.invI * (Vec2.Dot(r2, r2) - rt2 * rt2);
            c.massTangent = 1.0f /  kTangent;

            c.bias = -k_biasFactor * inv_dt * FMath.Min(0.0f, c.separation + k_allowedPenetration);

            if (World.accumulateImpulses)
            {
                // Apply normal + friction impulse
                Vec2 P = c.Pn * c.normal + c.Pt * tangent;

                body1.velocity -= body1.invMass * P;
                body1.angularVelocity -= body1.invI * Vec2.Cross(r1, P);

                body2.velocity += body2.invMass * P;
                body2.angularVelocity += body2.invI * Vec2.Cross(r2, P);
            }
        }
    }

    public void ApplyImpulse(Span<Body> bodies)
    {
        ref Body b1 = ref bodies[body1Ref];
        ref Body b2 = ref bodies[body2Ref];

        for (int i = 0; i < numContacts; ++i)
        {
            ref Contact c = ref contacts[i];
            c.r1 = c.position - b1.position;
            c.r2 = c.position - b2.position;

            // Relative velocity at contact
            Vec2 dv = b2.velocity + Vec2.Cross(b2.angularVelocity, c.r2) - b1.velocity - Vec2.Cross(b1.angularVelocity, c.r1);

            // Compute normal impulse
            float vn = Vec2.Dot(dv, c.normal);

            float dPn = c.massNormal * (-vn + c.bias);

            if (World.accumulateImpulses)
            {
                // Clamp the accumulated impulse
                float Pn0 = c.Pn;
                c.Pn = FMath.Max(Pn0 + dPn, 0.0f);
                dPn = c.Pn - Pn0;
            }
            else
            {
                dPn = FMath.Max(dPn, 0.0f);
            }

            // Apply contact impulse
            Vec2 Pn = dPn * c.normal;

            b1.velocity -= b1.invMass * Pn;
            b1.angularVelocity -= b1.invI * Vec2.Cross(c.r1, Pn);

            b2.velocity += b2.invMass * Pn;
            b2.angularVelocity += b2.invI * Vec2.Cross(c.r2, Pn);

            // Relative velocity at contact
            dv = b2.velocity + Vec2.Cross(b2.angularVelocity, c.r2) - b1.velocity - Vec2.Cross(b1.angularVelocity, c.r1);

            Vec2 tangent = Vec2.Cross(c.normal, 1.0f);
            float vt = Vec2.Dot(dv, tangent);
            float dPt = c.massTangent * (-vt);

            if (World.accumulateImpulses)
            {
                // Compute friction impulse
                float maxPt = friction * c.Pn;

                // Clamp friction
                float oldTangentImpulse = c.Pt;
                c.Pt = FMath.Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
                dPt = c.Pt - oldTangentImpulse;
            }
            else
            {
                float maxPt = friction * dPn;
                dPt = FMath.Clamp(dPt, -maxPt, maxPt);
            }

            // Apply contact impulse
            Vec2 Pt = dPt * tangent;

            b1.velocity -= b1.invMass * Pt;
            b1.angularVelocity -= b1.invI * Vec2.Cross(c.r1, Pt);

            b2.velocity += b2.invMass * Pt;
            b2.angularVelocity += b2.invI * Vec2.Cross(c.r2, Pt);
        }
    }

    // Unfortunately C# can't have fixed arrays for non-primitive types
    public Contact[] contacts = new Contact[MAX_POINTS];
    public int numContacts;

	BodyIndex body1Ref;
	BodyIndex body2Ref;

	// Combined friction
	float friction;
};