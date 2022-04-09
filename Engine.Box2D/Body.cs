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

namespace Engine.Box2D;

public readonly struct BodyIndex : IEquatable<BodyIndex>
{
    public static readonly BodyIndex Invalid = new(-1);

    public readonly int Value;

    public BodyIndex(int value)
    {
        Value = value;
    }

    public static implicit operator int(BodyIndex val) => val.Value;

    public static explicit operator BodyIndex(int val) => new(val);

    #region Equality

    public bool Equals(BodyIndex other)
    {
        return Value == other.Value;
    }

    public override bool Equals(object? obj)
    {
        return obj is BodyIndex other && Equals(other);
    }

    public override int GetHashCode()
    {
        return Value;
    }

    public static bool operator ==(BodyIndex left, BodyIndex right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(BodyIndex left, BodyIndex right)
    {
        return !left.Equals(right);
    }

    #endregion
}

struct Body : IEquatable<Body>
{
    public Body()
    {
        position = new Vec2(0.0f, 0.0f);
        rotation = 0.0f;
        velocity = new Vec2(0.0f, 0.0f);
        angularVelocity = 0.0f;
        force = new Vec2(0.0f, 0.0f);
        torque = 0.0f;
        friction = 0.2f;

        width = new Vec2(1.0f, 1.0f);
        mass = float.MaxValue;
        invMass = 0.0f;
        I = float.MaxValue;
        invI = 0.0f;
    }

    public void Set(in Vec2 w, float m)
    {
        position.Set(0.0f, 0.0f);
        rotation = 0.0f;
        velocity.Set(0.0f, 0.0f);
        angularVelocity = 0.0f;
        force.Set(0.0f, 0.0f);
        torque = 0.0f;
        friction = 0.2f;

        width = w;
        mass = m;

        if (mass < float.MaxValue)
        {
            invMass = 1.0f / mass;
            I = mass * (width.x * width.x + width.y * width.y) / 12.0f;
            invI = 1.0f / I;
        }
        else
        {
            invMass = 0.0f;
            I = float.MaxValue;
            invI = 0.0f;
        }
    }

    public void AddForce(in Vec2 f)
    {
        force += f;
    }

    public Vec2 position;
    public float rotation;

    public Vec2 velocity;
    public float angularVelocity;

    public Vec2 force;
    public float torque;

    public Vec2 width;

    public float friction;
    public float mass;
    public float invMass;
    public float I;
    public float invI;

    #region Equality

    public bool Equals(Body other)
    {
        return position.Equals(other.position)
               && rotation.Equals(other.rotation)
               && velocity.Equals(other.velocity)
               && angularVelocity.Equals(other.angularVelocity)
               && force.Equals(other.force)
               && torque.Equals(other.torque)
               && width.Equals(other.width)
               && friction.Equals(other.friction)
               && mass.Equals(other.mass)
               && invMass.Equals(other.invMass)
               && I.Equals(other.I)
               && invI.Equals(other.invI);
    }

    public override bool Equals(object? obj)
    {
        return obj is Body other && Equals(other);
    }

    public override int GetHashCode()
    {
        var hashCode = new HashCode();
        hashCode.Add(position);
        hashCode.Add(rotation);
        hashCode.Add(velocity);
        hashCode.Add(angularVelocity);
        hashCode.Add(force);
        hashCode.Add(torque);
        hashCode.Add(width);
        hashCode.Add(friction);
        hashCode.Add(mass);
        hashCode.Add(invMass);
        hashCode.Add(I);
        hashCode.Add(invI);
        return hashCode.ToHashCode();
    }

    public static bool operator ==(Body left, Body right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(Body left, Body right)
    {
        return !left.Equals(right);
    }

    #endregion
};